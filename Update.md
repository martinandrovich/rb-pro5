# rb-pro5 - 2019/10-02 - Daniel

Made a hello world avoid obstacle fuzzy controller.

### Struct

This struct contains the information of the closest obstacle for each sensor; straight, right left. Each sensor has 25 degrees of laser beam and the shortest range is contained in the struct, see the min_range method.

```
typedef struct closest_obstacle 
{
  float dir_delta;
  float range;
  closest_obstacle(float dir_delta_, float range_) : dir_delta(dir_delta_) , range(range_){};
} closest_obstacle;

min_range(cloest_obs_front, range_min, range_max, px_per_m, angle, im, msg, i, 0.19, -0.19);
```

### Demo

Only tested in the smallworld, should drive around in the first room.
