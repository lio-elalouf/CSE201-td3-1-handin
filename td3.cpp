#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>
#include <limits> // + inf double

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
    double* new_array;
    new_array = new double[new_size];
    //new_array[new_size] = {}; / better do the if
    for (int i=0 ; i< new_size ; i++) {
        if (i< length) {
        new_array[i]  = array[i];
        }
        else {
        new_array[i] = 0;
        }
    }
    delete array;
    return new_array;
}

double* shrink_array(double* array, int length, int new_size) {
    double* new_array;
    new_array = new double[new_size];
    for (int i=0 ; i< new_size ; i++) {
        new_array[i]  = array[i];
    }
    delete array;
    return new_array;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
 if (current_size == max_size) {
     array = extend_array(array, current_size, max_size+5);
     max_size += 5;
 }
     array[current_size] = element; // set the value of the last element
     current_size += 1;

     return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    if (current_size > 0) {
    current_size -= 1;
        if (max_size - current_size >= 5) { // distance (total nb of used elements, array maximum size)
            array = shrink_array(array, current_size, max_size-5);
            max_size -= 5;}
    }
        return array; // return it anyway
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;
  // add the first coordinates
  telemetry = append_to_array(t,telemetry, telemetry_current_size, telemetry_max_size);
  telemetry = append_to_array(x,telemetry, telemetry_current_size, telemetry_max_size);
  telemetry = append_to_array(y,telemetry, telemetry_current_size, telemetry_max_size);

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
      if (y >= 0) { // can not be under the ground
      telemetry = append_to_array(t,telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(x,telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(y,telemetry, telemetry_current_size, telemetry_max_size);
    }
    }
  }
  return hit_target;
}



void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
// Multiple telemetries are stored in "telemetries" (pointer to a pointer to doubles)
// telemetries : liste des telemetries
// tot_telemetries : nombre de projectile
// telemetries_sizes : taille de chacune des telemetries
// global_telemetry : listes once merged
// global_telemetry_current_size: total nb of elements stored in global_telemetry
// global_telemetry_max_size: total amount of elements that have been allocated


    global_telemetry_max_size = 0;
    global_telemetry_current_size = 0;
    global_telemetry = new double[global_telemetry_max_size];

    if (tot_telemetries == 0) {
      return;
    }

    double plus_inf = std::numeric_limits<double>::infinity();
    int depasser = 0;
    double** telemetries_ptr;
    telemetries_ptr = new double*[tot_telemetries];
    for (int i = 0; i < tot_telemetries; i++) {
      telemetries_ptr[i] = telemetries[i];
        if (telemetries_sizes[i] == 0) {
        telemetries_ptr[i] = &plus_inf;
        depasser+= 1;}
        }

    while(depasser < tot_telemetries) {
      double values[tot_telemetries];
      double min_value = *telemetries_ptr[0];
      int where_min_value = 0;
          for (int i = 0; i < tot_telemetries; i++) {
            values[i] = *telemetries_ptr[i];
            if (values[i] < min_value) {
              min_value = values[i];
              where_min_value = i;}}

      // copie de 3 valeur de telemetry
global_telemetry = append_to_array(*(telemetries_ptr[where_min_value]), global_telemetry,global_telemetry_current_size, global_telemetry_max_size);
global_telemetry = append_to_array(*(telemetries_ptr[where_min_value]+1), global_telemetry,global_telemetry_current_size, global_telemetry_max_size);
global_telemetry = append_to_array(*(telemetries_ptr[where_min_value]+2), global_telemetry,global_telemetry_current_size, global_telemetry_max_size);
      // avancer le pointeur correspondant à where_min_value
    (telemetries_ptr[where_min_value]) += 3;
      // si jamais le ptr dépasse la fin, le pointer vers plus_inf
    double* end = telemetries[where_min_value] + telemetries_sizes[where_min_value];
    if (telemetries_ptr[where_min_value] >= end ){
      telemetries_ptr[where_min_value] = &plus_inf;
    depasser+= 1;}
    }
delete[] telemetries_ptr;
}
