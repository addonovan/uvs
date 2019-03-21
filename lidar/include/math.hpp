#pragma once

/**
 * Converts the number in radians in the interval [-pi/2, pi/2] 
 * to degrees within the interval [-180, 180]. 
 *
 * This will also shift the angles around 180°, so that the 0° 
 * mark is straight forward.
 */
double rad2deg(double radians);

/**
 * Converts the degrees on the interval [-180, 180] to to the
 * interval [-pi/2, pi/2].
 */
double deg2rad(double degrees);

