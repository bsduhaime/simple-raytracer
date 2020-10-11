#ifndef RAYTRACE_H
#define RAYTRACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265358979323846

#define GAMMA 2.2
#define EXPOSURE 1.0
#define SAMPLES 4               // Number of samples taken for each light bounce's indirect light.

#define RAY_DIST_MIN 0.0001f    // Minimum range when a ray is considered self-intersecting.
#define RAY_DIST_MAX 1.0e30f    // Maximum range when a ray is considered infinite.
#define RAY_BOUNCE_MAX 3        // Number of times a ray can bounce before it's stopped.

#define SHAPE_MAX 16            // Maximum number of shapes allowed in a scene.
#define LIGHT_MAX 16            // Maximum number of lights allowed in a scene.

//
// VECTOR3
//

typedef struct Vector3{
    float x;
    float y;
    float z;
} Vector3;

int vec3_Create(Vector3* new_vec, float x, float y, float z);
int vec3_CreateSingle(Vector3* new_vec, float f);
Vector3 vec3_CreateCopy(Vector3* vec);

float vec3_LengthSquared(Vector3* vec);
float vec3_Length(Vector3* vec);

float vec3_Normalize(Vector3* vec);
Vector3 vec3_NormalizedCopy(Vector3* vec);

int vec3_AddVectors(Vector3* vec_a, Vector3* vec_b);
int vec3_SubVectors(Vector3* vec_a, Vector3* vec_b);
int vec3_MulVector(Vector3* vec_a, float f);
int vec3_DivVector(Vector3* vec_a, float f);

float vec3_DotProduct(Vector3* vec_a, Vector3* vec_b);
Vector3 vec3_CrossProduct(Vector3* vec_a, Vector3* vec_b);

Vector3 vec3_Reflect(Vector3* incoming, Vector3* normal);
Vector3 vec3_Refract(Vector3* incoming, Vector3* normal, float IOR);
Vector3 vec3_Rotate(Vector3* rotating_axis, Vector3* rotating_vec, float theta);

Vector3 vec3_Slerp(Vector3* start, Vector3* end, float t);

//
// VECTOR2
//

typedef struct Vector2{
    float u;
    float v;
} Vector2;

int vec2_Create(Vector2* new_vec, float u, float v);
int vec2_CreateSingle(Vector2* new_vec, float f);
Vector2 vec2_CreateCopy(Vector2* vec);

int vec2_AddVectors(Vector2* vec_a, Vector2* vec_b);
int vec2_SubVectors(Vector2* vec_a, Vector2* vec_b);
int vec2_MulVector(Vector2* vec_a, float f);
int vec2_DivVector(Vector2* vec_a, float f);

//
// 3X3 MATRIX
//

typedef struct Matrix3{
    float data[9];
} Matrix3;

int mat3_Create(Matrix3* mat, float val);
Matrix3 mat3_CreateCopy(Matrix3* mat);

Matrix3 mat3_GetInverseMatrix(Matrix3* mat);
int mat3_AddMatrices(Matrix3* mat_a, Matrix3* mat_b);
int mat3_MultiplyMatrices(Matrix3* mat_a, Matrix3* mat_b);
int mat3_MultiplyMatrixByValue(Matrix3* mat, float val);
Vector3 mat3_MultiplyMatrixAndVector(Matrix3* mat, Vector3* vec);
int mat3_CreateRotationMatrix(Matrix3* mat, Vector3* vec_a, Vector3* vec_b);

//
// COLOR
//

typedef struct Color{
    float r;
    float g;
    float b;
} Color;

int col_ClampColor(Color* col);
int col_Create(Color* new_col, float r, float g, float b);
int col_CreateGrayscale(Color* new_col, float s);
Color col_CreateCopy(Color* col);

int col_AddColors(Color* col_a, Color* col_b);
int col_MulColors(Color* col_a, Color* col_b);
int col_MulColorByValue(Color* col, float val);

int col_ApplyGammaCorrection(Color* col, float exposure, float gamma);

int col_GetColorDataLength(Color* col);
int col_ReadColor(char* dest_str, Color* col);

//
// MATERIAL
//

typedef struct Material{
    Color albedo;
    float diffuse;
    float specular;
    float ambient;
    float reflectance;
} Material;

int mat_Create(Material* new_mat, Color* alb, float diff, float spec, float amb, float ref);
Material mat_CreateCopy(Material* mat);

//
// RAY
//

typedef struct Ray{
    Vector3 origin;
    Vector3 dir;
    float t_max;
} Ray;

int ray_Create(Ray* new_ray, Vector3* origin, Vector3* dir, float t_max);
Ray ray_CreateCopy(Ray* ray);

Vector3 ray_GetPointOnRay(Ray* ray, float t);

//
// SHAPES AND INTERSECTIONS
//

enum ShapeTypes{
    EMPTY,
    PLANE,
    SPHERE
};

typedef struct Shape{
    enum ShapeTypes shape_type;
    // Universal variables
    Vector3 pos;
    Material mat;
    // Shape specific variables
    Vector3 plane_normal;
    float sphere_radius;
} Shape;

typedef struct Intersection{
    Ray ray;
    float t;
    Shape shape;
    Material mat;
} Intersection;

Shape shape_CreateCopy(Shape* shape);
int shape_CreateEmpty(Shape* shape);
int shape_CreatePlane(Shape* shape, Material* mat, Vector3* pos, Vector3* normal);
int shape_CreateSphere(Shape* shape, Material* mat, Vector3* pos, float radius);

int shape_FindIntersection(Shape* shape, Intersection* result_intsec);
int shape_FindPlaneIntersection(Shape* plane, Intersection* result_intsec);
int shape_FindSphereIntersection(Shape* sphere, Intersection* result_intsec);

int shape_DoesIntersect(Shape* shape, Ray* ray);
int shape_DoesPlaneIntersect(Shape* plane, Ray* ray);
int shape_DoesSphereIntersect(Shape* sphere, Ray* ray);

int intsec_Create(Intersection* new_intsec, Ray* ray);
Intersection intsec_CreateCopy(Intersection* intsec);

int intsec_HasIntersected(Intersection* intsec);
Vector3 intsec_GetIntersectPosition(Intersection* intsec);
Vector3 intsec_GetIntersectNormal(Intersection* intsec);

//
// LIGHT
//

typedef struct Light{
    Vector3 origin;
    Color color;
    float power;
} Light;

int light_Create(Light* new_light, Vector3* origin, Color* color, float power);
Light light_CreateCopy(Light* light);

Vector3 light_GetBiasedDirectionInHemisphere(Intersection* intsec, float* probability);
Vector3 light_GetDirectionInHemisphere(Intersection* intsec, float* probability);

Color light_GetDirectLightContribution(Light* light, Intersection* source_intsec, Vector3* cam_loc);

//
// CAMERA
//

typedef struct Camera{
    Vector3 origin;
    Vector3 forward;
    Vector3 up;
    Vector3 right;
    float height;
    float width;
} Camera;

int cam_Create(Camera* new_cam, Vector3* origin, Vector3* target, Vector3* up, float fov, float asp_ratio);
Camera cam_CreateCopy(Camera* cam);

Ray cam_MakeRay(Camera* cam, Vector2* screen_point);

//
// SCENE
//

typedef struct Scene{
    Shape shapes[SHAPE_MAX];
    int set_shapes_size;
    Light lights[LIGHT_MAX];
    int set_lights_size;
} Scene;

int scene_Create(Scene* new_scene);
Scene scene_CreateCopy(Scene* scene);
int scene_AddShape(Scene* scene, Shape* shape);
int scene_AddLight(Scene* scene, Light* light);
int scene_FindIntersections(Scene* scene, Intersection* result_intsec);
int scene_HasIntersection(Scene* scene, Ray* ray);

int scene_HasShadowIntersection(Scene* scene, Light* light, Intersection* source);

//
// IMAGE (PPM Format)
//

typedef struct Image{
    int width;
    int height;
    int data_size;
    Color* data;
} Image;

int img_Create(Image* img, int w, int h);

int img_GetWidth(Image* img);
int img_GetHeight(Image* img);

Color* img_GetPixel(Image* img, int x, int y);
int img_SetPixel(Color* pixel, Color val);
Vector2 img_GetPixelCoord(Image* img, int arr_pos);

int img_SaveImage(Image* img, const char* dir);
int img_Destroy(Image* img);

#endif