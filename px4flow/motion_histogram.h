/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Jonas Kuehne
 */

#ifndef __MOTION_HISTOGRAM_H__
#define __MOTION_HISTOGRAM_H__

#include "pmsis.h"
#include "flow.h"

typedef struct flow_f32
{
    float x;
    float y;
} flow_f32_t;

/**
 * @brief Compute averaged optical flow estimates from multiple predictions following the approach described in
 *        https://ieeexplore.ieee.org/abstract/document/6630805.
 *
 * @param flow_estimate    averaged flow prediction using the histogram approach
 * @param estimates_array  array containing the flow predictions for multiple points of interest
 */
void estimate_motion_histogram(flow_f32_t* flow_estimate, signed_coordinates_t* estimates_array);

#endif /* __MOTION_HISTOGRAM_H__ */
