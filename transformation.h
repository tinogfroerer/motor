#pragma once

// This is the size the image has in the table plane, not the actual size of the table
// However, approximating this with the table size should be fine.
const float size_x = 1.5f;
const float size_y = 0.75f;

// Variables to adjust for the fact, that the chuck blobs are not in the table plain like
// the puck, but slightly elevated
const float chuck_height = 0.035f; // Height of the blobs depicting the chuck
const float camera_height = 1.2f;
const float height_correction_factor = 1.0f - chuck_height / camera_height;

// Chuck blobs and the chuck center are apart from each other by chuck_x_offset
const float chuck_x_offset = 0.035f; // chuckCenter - mean(BlobCenters)_x (careful with the sign)

// Distance between the two chuck blobs. This is the distance the camera thinks they are apart,
// not the distance they are actually apart.
const float chuck_blobs_apparent_y_distance = 0.052f; 
// Circle radius in which the second blob is searched for
const float r_tolerance = 0.006f;
