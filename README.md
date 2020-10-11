# simple-raytracer
A simple raytracer written in C.

A simple, multi-threaded raytracer. Tweak the performance with the following constants:

raytrace.h:
GAMMA 2.2
EXPOSURE 1.0

SAMPLES 4               // Number of samples taken for each light bounce's indirect light.

RAY_BOUNCE_MAX 3        // Number of times a ray can bounce before it's stopped.

SHAPE_MAX 16            // Maximum number of shapes allowed in a scene.

LIGHT_MAX 16            // Maximum number of lights allowed in a scene.


raytrace.c:
THREAD_COUNT 7          // Number of threads to create when multi-threading.


The resulting file will be called "ray_result.ppm" in the same directory as the executable.

Build using gcc: "gcc -g -Wall raytrace.c -lm -lpthread -o raytrace"
