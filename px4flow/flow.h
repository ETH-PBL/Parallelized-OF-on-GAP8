/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Jonas Kuehne
 */

#ifndef FLOW_H
#define FLOW_H

#include "pmsis.h"

#define IMAGE_WIDTH 64
#define SEARCH_SIZE 4
#define TILE_SIZE 8
#define NUM_BLOCKS 8
#define NUM_CORES 8
#define HIST_SIZE (2*(2*SEARCH_SIZE +1)+1)

typedef struct unsigned_coordinates
{
    uint16_t x;
    uint16_t y;
} unsigned_coordinates_t;

typedef struct signed_coordinates
{
    int8_t x;
    int8_t y;
} signed_coordinates_t;

struct processing_arguments
{
    uint8_t* frame1;
    uint8_t* frame2;
    struct unsigned_coordinates* coordinates;
    uint8_t total_num_points;
    struct signed_coordinates* results;
};

/**
 * @brief Compute optical flow estimates for predefined coordinates in an image frame.
 *
 * @param arg        arguments for the flow computation of the type `struct processing_arguments`
 * @param num_cores  number of cores that are used for the parallel computation of the flow predictions
 */
void flow(void* arg, uint8_t num_cores);

#endif /* FLOW_H */