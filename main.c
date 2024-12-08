/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Jonas Kuehne
 */

#include "pmsis.h"
#include "px4flow/flow.h"
#include "px4flow/motion_histogram.h"

#include "test_input.h"

#define ONE_CORE 1
#define EIGHT_CORES 8

uint8_t in_buffer [IMAGE_WIDTH*IMAGE_WIDTH];

/* Set the performance metric that is to be evaluated. */
char* performance_metrics[] = {
    "Active Cycles",                           /* PI_PERF_ACTIVE_CYCLES */
    "Instructions Executed",                   /* PI_PERF_INSTR */
    "Load Data Stalls",                        /* PI_PERF_LD_STALL */
    "Jump Register Stalls",                    /* PI_PERF_JR_STALL */
    "Instruction Cache Misses",                /* PI_PERF_IMISS */
    "Data Loads Executed",                     /* PI_PERF_LD */
    "Data Stores Executed",                    /* PI_PERF_ST */
    "Unconditional Jumps Executed",            /* PI_PERF_JUMP */
    "Number of Branches (taken and not taken)",/* PI_PERF_BRANCH */
    "Branches Taken",                          /* PI_PERF_BTAKEN */
    "Compressed Instructions Executed",        /* PI_PERF_RVC */
    "Memory Loads to EXT Executd",             /* PI_PERF_LD_EXT */
    "Memory Loads from EXT Executd",           /* PI_PERF_ST_EXT */
    "Cycles Used for Loads from EXT",          /* PI_PERF_LD_EXT_CYC */
    "Cycles Used for Stores to EXT",           /* PI_PERF_ST_EXT_CYC */
    "Cycles Wasted due to TCDM Contention"};   /* PI_PERF_TCDM_CONT */

/* The bit offsets are defined in the GAP SDK under install/GAP8_V3/include/pmsis/chips/gap8/perf.h */
uint8_t PERFORMANCE_COUNTER = PI_PERF_INSTR;
uint32_t performance_value = 0;
uint32_t tim_cycles = 0;

uint8_t SINGLE_CORE = 0;


struct arguments_to_cluster
{
    uint8_t* buffer_frame1;
    uint8_t* buffer_frame2;
    pi_cl_dma_copy_t* copy_settings;
};

/* Task executed by cluster cores. */
void cluster_flow(void *arg)
{
    flow(arg, EIGHT_CORES);
}

void single_core_flow(void *arg)
{
    flow(arg, ONE_CORE);
}

/* Cluster main entry, executed by core 0. */
void cluster_delegate(void *arg)
{
    printf("Cluster master core entry\n");
    /* DMA copy new image to L1 cache */
    printf("Performing DMA copy of new frame.\n");
    struct arguments_to_cluster* arguments = (struct arguments_to_cluster*) arg;

    printf("Starting performance measurement.\n");

    pi_perf_reset();
    pi_perf_conf(1 << PERFORMANCE_COUNTER);
    pi_perf_start();

    pi_cl_dma_memcpy(arguments->copy_settings);

    /* Calculate position of flow vectors */
    uint8_t smallest_pixel = SEARCH_SIZE + 1;
    uint8_t biggest_pixel = IMAGE_WIDTH - (SEARCH_SIZE + 1) - TILE_SIZE;
    uint8_t pixel_step_size = ((biggest_pixel-smallest_pixel)/ ((uint8_t) NUM_BLOCKS))+1;

    struct unsigned_coordinates points_of_interest [NUM_BLOCKS*NUM_BLOCKS];
    struct signed_coordinates results [NUM_BLOCKS*NUM_BLOCKS];

    uint8_t num_points = 0;
    for(uint16_t y = smallest_pixel; y < biggest_pixel; y += pixel_step_size)
    {
        for(uint16_t x = smallest_pixel; x < biggest_pixel; x += pixel_step_size)
        {
            struct unsigned_coordinates coord = {x,y};
            points_of_interest[num_points] =  coord;
            num_points++;
        }
    }

    struct processing_arguments args = {arguments->buffer_frame1,arguments->buffer_frame2,points_of_interest, num_points,results};

    /* Wait for DMA completion. */
    pi_cl_dma_wait(arguments->copy_settings);

    /* Task dispatch to cluster cores. */
    if(SINGLE_CORE)
    {
        /* Single core */
        single_core_flow((void*)&args);
    }
    else
    {
        /* Multi core */
        pi_cl_team_fork(pi_cl_cluster_nb_cores(), cluster_flow, (void*)&args);
    }

    flow_f32_t flow_estimation;
    estimate_motion_histogram(&flow_estimation, results);

    pi_perf_stop();
    printf("Stopping performance measurement.\n");

    tim_cycles = pi_perf_read(PI_PERF_CYCLES);
    performance_value = pi_perf_read(PERFORMANCE_COUNTER);

    printf("Flow x %.4f, flow y %.4f \n", flow_estimation.x, flow_estimation.y);
    printf("Cluster master core exit\n");
}

void program_entry(void)
{
    printf("Entering main controller\n");
    uint32_t errors = 0;
    uint32_t core_id = pi_core_id(), cluster_id = pi_cluster_id();

    struct pi_device cluster_dev = {0};
    struct pi_cluster_conf cl_conf = {0};

    /* Setting up performance counters. */
    pi_perf_conf(1 << PI_PERF_CYCLES | 1 << PERFORMANCE_COUNTER);

    /* Init cluster configuration structure. */
    pi_cluster_conf_init(&cl_conf);
    cl_conf.id = 0;                /* Set cluster ID. */
    /* Configure & open cluster. */
    pi_open_from_conf(&cluster_dev, &cl_conf);
    if (pi_cluster_open(&cluster_dev))
    {
        printf("Cluster open failed !\n");
        pmsis_exit(-1);
    }

    printf("Loading new image\n");
    for(uint16_t i = 0; i < (IMAGE_WIDTH*IMAGE_WIDTH); ++i)
    {
        in_buffer[i] = sample_frame1[i];
    }

    /* Allocate L1 buffers for both images. */
    uint8_t* l1_buffer_frame1 = (uint8_t*) pmsis_l1_malloc((uint32_t) (IMAGE_WIDTH*IMAGE_WIDTH));
    uint8_t* l1_buffer_frame2 = (uint8_t*) pmsis_l1_malloc((uint32_t) (IMAGE_WIDTH*IMAGE_WIDTH));

    for(uint16_t i = 0; i < (IMAGE_WIDTH*IMAGE_WIDTH); ++i)
    {
        l1_buffer_frame2[i] = sample_frame2[i];
    }

    /* Prepare cluster task it to cluster. */
    struct pi_cluster_task cl_task = {0};
    cl_task.entry = cluster_delegate;
    pi_cl_dma_copy_t copy;
    struct arguments_to_cluster args = {l1_buffer_frame1,l1_buffer_frame2,&copy};
    cl_task.arg = (&args);

    _Bool end_of_stream = 0;
    while(!end_of_stream)
    {
        copy.dir = PI_CL_DMA_DIR_EXT2LOC;
        copy.merge = 0;
        copy.size = (uint16_t) (IMAGE_WIDTH*IMAGE_WIDTH);
        copy.id = 0;
        copy.ext = (uint32_t) in_buffer;
        /* ToDo: Alternate target location. */
        copy.loc = (uint32_t) l1_buffer_frame1;

        pi_cluster_send_task_to_cl(&cluster_dev, &cl_task);

        end_of_stream = 1;
    }


    printf("Timer: %d cycles\n", tim_cycles);
    printf("Performance Counter [%s] is %d.\n", performance_metrics[PERFORMANCE_COUNTER], performance_value);
    int32_t cur_cl_freq = pi_freq_get(PI_FREQ_DOMAIN_CL);
    float running_time = (float) performance_value/ ((float) cur_cl_freq);
    printf("Running time %fs \n", running_time);

    pi_cluster_close(&cluster_dev);

    printf("Test success !\n");

    pmsis_exit(errors);
}

/* Program Entry. */
int main(void)
{
    printf("\n\n\t *** Optical Flow Example Project ***\n\n");
    return pmsis_kickoff((void *) program_entry);
}

