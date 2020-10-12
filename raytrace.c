#include "raytrace.h"

#include <pthread.h>
#define THREAD_COUNT 7

//
// VECTOR3
//

int vec3_Create(Vector3* new_vec, float x, float y, float z){
    new_vec->x = x;
    new_vec->y = y;
    new_vec->z = z;
    return 1;
}

int vec3_CreateSingle(Vector3* new_vec, float f){
    new_vec->x = f;
    new_vec->y = f;
    new_vec->z = f;
    return 1;
}

Vector3 vec3_CreateCopy(Vector3* vec){
    Vector3 new_vec;
    new_vec.x = vec->x;
    new_vec.y = vec->y;
    new_vec.z = vec->z;
    return new_vec;
}

float vec3_LengthSquared(Vector3* vec){
    float x = vec->x;
    float y = vec->y;
    float z = vec->z;
    float len_sq = (x * x) + (y * y) + (z * z);
    return len_sq;
}

float vec3_Length(Vector3* vec){
    float len_sq = vec3_LengthSquared(vec);
    float len = sqrtf(len_sq);
    return len;
}

float vec3_Normalize(Vector3* vec){
    float len = vec3_Length(vec);
    if(len == 0.0f){
        return 0.0f;
    }
    float true_x = vec->x;
    float true_y = vec->y;
    float true_z = vec->z;
    vec->x = true_x / len;
    vec->y = true_y / len;
    vec->z = true_z / len;
    return len;
}

Vector3 vec3_NormalizedCopy(Vector3* vec){
    Vector3 vec_copy;
    float len = vec3_Length(vec);
    float true_x = vec->x;
    float true_y = vec->y;
    float true_z = vec->z;
    vec_copy.x = true_x / len;
    vec_copy.y = true_y / len;
    vec_copy.z = true_z / len;
    return vec_copy;
}

int vec3_AddVectors(Vector3* vec_a, Vector3* vec_b){
    vec_a->x = vec_a->x + vec_b->x;
    vec_a->y = vec_a->y + vec_b->y;
    vec_a->z = vec_a->z + vec_b->z;
    return 1;
}

int vec3_SubVectors(Vector3* vec_a, Vector3* vec_b){
    vec_a->x = vec_a->x - vec_b->x;
    vec_a->y = vec_a->y - vec_b->y;
    vec_a->z = vec_a->z - vec_b->z;
    return 1;
}

int vec3_MulVector(Vector3* vec_a, float f){
    vec_a->x = vec_a->x * f;
    vec_a->y = vec_a->y * f;
    vec_a->z = vec_a->z * f;
    return 1;
}

int vec3_DivVector(Vector3* vec_a, float f){
    if(f == 0.0f){
        return 0;
    }
    vec_a->x = vec_a->x / f;
    vec_a->y = vec_a->y / f;
    vec_a->z = vec_a->z / f;
    return 1;
}

float vec3_DotProduct(Vector3* vec_a, Vector3* vec_b){
    float comp1 = vec_a->x * vec_b->x;
    float comp2 = vec_a->y * vec_b->y;
    float comp3 = vec_a->z * vec_b->z;
    return (comp1 + comp2 + comp3);
}

Vector3 vec3_CrossProduct(Vector3* vec_a, Vector3* vec_b){
    Vector3 cross_vec;
    cross_vec.x = (vec_a->y * vec_b->z) - (vec_a->z * vec_b->y);
    cross_vec.y = (vec_a->z * vec_b->x) - (vec_a->x * vec_b->z);
    cross_vec.z = (vec_a->x * vec_b->y) - (vec_a->y * vec_b->x);
    return cross_vec;
}

Vector3 vec3_Reflect(Vector3* incoming, Vector3* normal){
    Vector3 norm_comp = vec3_CreateCopy(normal);
    float dot_prod = vec3_DotProduct(incoming, normal);
    vec3_MulVector(&norm_comp, 2.0f * dot_prod);
    Vector3 reflection = vec3_CreateCopy(incoming);
    vec3_SubVectors(&reflection, &norm_comp);

    return reflection;
}

Vector3 vec3_Refract(Vector3* incoming, Vector3* normal, float IOR){
    Vector3 norm_copy = vec3_CreateCopy(normal);
    Vector3 incoming_copy = vec3_CreateCopy(incoming);
    float dot_prod = vec3_DotProduct(normal, incoming);
    float etai = 1.0f;
    float etat = IOR;
    if(dot_prod < 0.0f){
        dot_prod *= -1.0f;
    } else {
        float temp_etai = etai;
        etai = etat;
        etat = temp_etai;
        vec3_MulVector(&norm_copy, -1.0f);
    }
    float eta = etai / etat;
    float k = 1.0f - powf(eta, 2.0f) * (1.0f - powf(dot_prod, 2.0f));
    if(k < 0.0f){
        vec3_Create(&incoming_copy, 0.0f, 0.0f, 0.0f);
    } else {
        vec3_MulVector(&incoming_copy, eta);
        vec3_MulVector(&norm_copy, (eta * dot_prod - sqrtf(k)));
        vec3_AddVectors(&incoming_copy, &norm_copy);
    }
    return incoming_copy;
}

Vector3 vec3_Rotate(Vector3* rotating_axis, Vector3* rotating_vec, float theta){
    // Implemented using Rodrigues' formula
    Vector3 result = vec3_CreateCopy(rotating_vec);
    vec3_MulVector(&result, cosf(theta));

    Vector3 comp2 = vec3_CrossProduct(rotating_axis, rotating_vec);
    vec3_MulVector(&comp2, sinf(theta));

    Vector3 comp3 = vec3_CreateCopy(rotating_axis);
    float dot_prod = vec3_DotProduct(rotating_axis, rotating_vec);
    vec3_MulVector(&comp3, dot_prod * (1.0f - cosf(theta)));

    vec3_AddVectors(&result, &comp2);
    vec3_AddVectors(&result, &comp3);

    return result;
}

Vector3 vec3_Slerp(Vector3* start, Vector3* end, float t){
    Vector3 result;
    Vector3 start_norm = vec3_CreateCopy(start);
    vec3_Normalize(&start_norm);
    Vector3 end_norm = vec3_CreateCopy(end);
    vec3_Normalize(&end_norm);
    
    float dot_prod = vec3_DotProduct(&start_norm, &end_norm);
    /*
    if(dot_prod < 0.0f){
        vec3_MulVector(&end_norm, -1.0f);
        dot_prod = -1.0f * dot_prod;
    }
    */
    const float DOT_THRESHOLD = 0.9995f;
    if(dot_prod > DOT_THRESHOLD){    // Unsafe to slerp. Linear interpolate instead.
        result = vec3_CreateCopy(&start_norm);
        Vector3 vec_diff = vec3_CreateCopy(&end_norm);
        vec3_SubVectors(&vec_diff, &start_norm);
        vec3_MulVector(&vec_diff, t);
        vec3_AddVectors(&result, &vec_diff);
        vec3_Normalize(&result);
        return result;
    }

    // Safe to slerp.
    float omega = acosf(dot_prod);
    float comp1 = (sinf(1.0f - t) * omega) / (sinf(omega));
    float comp2 = (sinf(omega * t)) / (sinf(omega));
    vec3_MulVector(&start_norm, comp1);
    vec3_MulVector(&end_norm, comp2);
    result = vec3_CreateCopy(&start_norm);
    vec3_AddVectors(&result, &end_norm);

    return result;
}

//
// VECTOR2
//

int vec2_Create(Vector2* new_vec, float u, float v){
    new_vec->u = u;
    new_vec->v = v;
    return 1;
}

int vec2_CreateSingle(Vector2* new_vec, float f){
    new_vec->u = f;
    new_vec->v = f;
    return 1;
}

Vector2 vec2_CreateCopy(Vector2* vec){
    Vector2 new_vec;
    new_vec.u = vec->u;
    new_vec.v = vec->v;
    return new_vec;
}

int vec2_AddVectors(Vector2* vec_a, Vector2* vec_b){
    vec_a->u = vec_a->u + vec_b->u;
    vec_a->v = vec_a->v + vec_b->v;
    return 1;
}

int vec2_SubVectors(Vector2* vec_a, Vector2* vec_b){
    vec_a->u = vec_a->u - vec_b->u;
    vec_a->v = vec_a->v - vec_b->v;
    return 1;
}

int vec2_MulVector(Vector2* vec_a, float f){
    vec_a->u = vec_a->u * f;
    vec_a->v = vec_a->v * f;
    return 1;
}

int vec2_DivVector(Vector2* vec_a, float f){
    if(f == 0.0f){
        return 0;
    }
    vec_a->u = vec_a->u / f;
    vec_a->v = vec_a->v / f;
    return 1;
}

//
// MATRIX3
//

int mat3_Create(Matrix3* mat, float val){
    for(int i = 0; i < 9; i++){
        (mat->data)[i] = val;
    }
    return 1;
}

Matrix3 mat3_CreateCopy(Matrix3* mat){
    Matrix3 new_mat;
    for(int i = 0; i < 9; i++){
        (new_mat.data)[i] = (mat->data)[i];
    }
    return new_mat;
}

Matrix3 mat3_GetInverseMatrix(Matrix3* mat){
    Matrix3 minors;
    Matrix3 inv_mat;
    float* m = mat->data;
    // Matrix of minors
    minors.data[0] = (m[4] * m[8]) - (m[5] * m[7]);
    minors.data[1] = (m[3] * m[8]) - (m[5] * m[6]);
    minors.data[2] = (m[3] * m[7]) - (m[4] * m[6]);
    minors.data[3] = (m[1] * m[8]) - (m[2] * m[7]);
    minors.data[4] = (m[0] * m[8]) - (m[2] * m[6]);
    minors.data[5] = (m[0] * m[7]) - (m[1] * m[6]);
    minors.data[6] = (m[1] * m[5]) - (m[2] * m[4]);
    minors.data[7] = (m[0] * m[5]) - (m[2] * m[3]);
    minors.data[8] = (m[0] * m[4]) - (m[1] * m[3]);
    // Determinant of the matrix
    float mat_det = (m[0] * minors.data[0]) + (-1.0f * m[1] * minors.data[1]) + (m[2] * minors.data[2]);
    // Transposed adjucate matrix
    inv_mat.data[0] = 1.0f * minors.data[0] * (1.0f / mat_det);
    inv_mat.data[1] = -1.0f * minors.data[3] * (1.0f / mat_det);
    inv_mat.data[2] = 1.0f * minors.data[6] * (1.0f / mat_det);
    inv_mat.data[3] = -1.0f * minors.data[1] * (1.0f / mat_det);
    inv_mat.data[4] = 1.0f * minors.data[4] * (1.0f / mat_det);
    inv_mat.data[5] = -1.0f * minors.data[7] * (1.0f / mat_det);
    inv_mat.data[6] = 1.0f * minors.data[2] * (1.0f / mat_det);
    inv_mat.data[7] = -1.0f * minors.data[5] * (1.0f / mat_det);
    inv_mat.data[8] = 1.0f * minors.data[8] * (1.0f / mat_det);
    return inv_mat;
}

int mat3_AddMatrices(Matrix3* mat_a, Matrix3* mat_b){
    for(int i = 0; i < 9; i++){
        (mat_a->data)[i] += (mat_b->data)[i];
    }
    return 1;
}

int mat3_MultiplyMatrices(Matrix3* mat_a, Matrix3* mat_b){
    Matrix3 final_mat;
    float* a = mat_a->data;
    float* b = mat_b->data;
    final_mat.data[0] = (a[0] * b[0]) + (a[1] * b[3]) + (a[2] * b[6]);
    final_mat.data[1] = (a[0] * b[1]) + (a[1] * b[4]) + (a[2] * b[7]);
    final_mat.data[2] = (a[0] * b[2]) + (a[1] * b[5]) + (a[2] * b[8]);
    final_mat.data[3] = (a[3] * b[0]) + (a[4] * b[3]) + (a[5] * b[6]);
    final_mat.data[4] = (a[3] * b[1]) + (a[4] * b[4]) + (a[5] * b[7]);
    final_mat.data[5] = (a[3] * b[2]) + (a[4] * b[5]) + (a[5] * b[8]);
    final_mat.data[6] = (a[6] * b[0]) + (a[7] * b[3]) + (a[8] * b[6]);
    final_mat.data[7] = (a[6] * b[1]) + (a[7] * b[4]) + (a[8] * b[7]);
    final_mat.data[8] = (a[6] * b[2]) + (a[7] * b[5]) + (a[8] * b[8]);
    for(int i = 0; i < 9; i++){
        (mat_a->data)[i] = (final_mat.data)[i];
    }
    return 1;
}

int mat3_MultiplyMatrixByValue(Matrix3* mat, float val){
    for(int i = 0; i < 9; i++){
        (mat->data)[i] *= val;
    }
    return 1;
}

Vector3 mat3_MultiplyMatrixAndVector(Matrix3* mat, Vector3* vec){
    Vector3 new_vec;
    float* m = mat->data;
    new_vec.x = (vec->x * m[0]) + (vec->y * m[1]) + (vec->z * m[2]);
    new_vec.y = (vec->x * m[3]) + (vec->y * m[4]) + (vec->z * m[5]);
    new_vec.z = (vec->x * m[6]) + (vec->y * m[7]) + (vec->z * m[8]);
    return new_vec;
}

int mat3_CreateRotationMatrix(Matrix3* mat, Vector3* vec_a, Vector3* vec_b){
    Vector3 vec_a_norm = vec3_NormalizedCopy(vec_a);
    Vector3 vec_b_norm = vec3_NormalizedCopy(vec_b);
    float dot_prod = vec3_DotProduct(&vec_a_norm, &vec_b_norm);
    float rot_angle = acosf(dot_prod);

    Matrix3 rot_mat;
    mat3_Create(&rot_mat, 0.0f);
    rot_mat.data[0] = 1.0f;
    rot_mat.data[4] = 1.0f;
    rot_mat.data[8] = 1.0f;

    Vector3 rot_axis;
    const float ANGLE_THRESHOLD = 0.005f;
    if(rot_angle < ANGLE_THRESHOLD){
        for(int i = 0; i < 9; i++){
            (mat->data)[i] = (rot_mat.data)[i];
        }
        return 1;
    } else if(PI - rot_angle < ANGLE_THRESHOLD){
        Vector3 basis_x, basis_y, basis_z;
        vec3_Create(&basis_x, 1.0f, 0.0f, 0.0f);
        vec3_Create(&basis_y, 0.0f, 1.0f, 0.0f);
        vec3_Create(&basis_z, 0.0f, 0.0f, 1.0f);
        if(vec_a->x < vec_a->y && vec_a->x < vec_a->z){
            rot_axis = vec3_CrossProduct(vec_a, &basis_x);
        } else if(vec_a->y < vec_a->x && vec_a->y < vec_a->z){
            rot_axis = vec3_CrossProduct(vec_a, &basis_y);
        } else {
            rot_axis = vec3_CrossProduct(vec_a, &basis_z);
        }
    } else {
        rot_axis = vec3_CrossProduct(vec_a, vec_b);
    }
    vec3_Normalize(&rot_axis);

    Matrix3 x_skew_sym;
    mat3_Create(&x_skew_sym, 0.0f);
    x_skew_sym.data[1] = -1.0f * rot_axis.z;
    x_skew_sym.data[2] = rot_axis.y;
    x_skew_sym.data[3] = rot_axis.z;
    x_skew_sym.data[5] = -1.0f * rot_axis.x;
    x_skew_sym.data[6] = -1.0f * rot_axis.y;
    x_skew_sym.data[7] = rot_axis.x;

    Matrix3 comp1 = mat3_CreateCopy(&x_skew_sym);
    mat3_MultiplyMatrixByValue(&comp1, sinf(rot_angle));

    Matrix3 comp2 = mat3_CreateCopy(&x_skew_sym);
    mat3_MultiplyMatrices(&comp2, &x_skew_sym);
    mat3_MultiplyMatrixByValue(&comp2, (1.0f - cosf(rot_angle)));

    mat3_AddMatrices(&rot_mat, &comp1);
    mat3_AddMatrices(&rot_mat, &comp2);

    for(int i = 0; i < 9; i++){
        (mat->data)[i] = (rot_mat.data)[i];
    }
    return 1;
}

//
// RAYS
//

int ray_Create(Ray* new_ray, Vector3* origin, Vector3* dir, float t_max){
    if(t_max <= 0.0f){
        new_ray->t_max = RAY_DIST_MAX;
    } else {
        new_ray->t_max = t_max;
    }
    new_ray->origin = vec3_CreateCopy(origin);
    new_ray->dir = vec3_CreateCopy(dir);
    return 1;
}

Ray ray_CreateCopy(Ray* ray){
    Ray new_ray;
    new_ray.dir = ray->dir;
    new_ray.origin = ray->origin;
    new_ray.t_max = ray->t_max;
    return new_ray;
}

Vector3 ray_GetPointOnRay(Ray* ray, float t){
    Vector3 point = vec3_CreateCopy(&(ray->dir));
    vec3_MulVector(&point, t);
    vec3_AddVectors(&point, &(ray->origin));
    return point;
}

//
// LIGHTS
//

int light_Create(Light* new_light, Vector3* origin, Color* color, float power){
    new_light->origin = vec3_CreateCopy(origin);
    new_light->color = col_CreateCopy(color);
    new_light->power = power;
    return 1;
}

Light light_CreateCopy(Light* light){
    Light new_light;
    new_light.origin = vec3_CreateCopy(&(light->origin));
    new_light.color = col_CreateCopy(&(light->color));
    new_light.power = light->power;
    return new_light;
}

Vector3 light_GetBiasedDirectionInHemisphere(Intersection* intsec, float* probability){
    Vector3 normal = intsec_GetIntersectNormal(intsec);
    float mat_reflect = sqrtf((intsec->mat).reflectance);
    // Get the perfect ray reflection
    Vector3 incoming_dir = (*intsec).ray.dir;
    Vector3 incoming_reflected = vec3_Reflect(&incoming_dir, &normal);
    vec3_Normalize(&incoming_reflected);
    // Choose a random direction in a hemisphere with varaince augmented by reflection
    Vector3 unit_rand_dir;
    float u1 = (float)rand() * (1.0f - mat_reflect) / (float)RAND_MAX;
    float u2 = (float)rand() * (1.0f - mat_reflect) / (float)RAND_MAX;
    float rad = sqrtf(u1);
    float theta = 2.0f * PI * u2;
    float x = rad * cosf(theta);
    float y = rad * sinf(theta);
    float z = sqrtf(1.0f - u1);
    if(1.0f - u1 < 0.0f){
        z = sqrtf(0.0f);
    }
    vec3_Create(&unit_rand_dir, x, y, z);
    // Rotate "unit_rand_dir" into the same basis as the reflection vector.
    Vector3 hemi_normal;
    vec3_Create(&hemi_normal, 0.0f, 0.0f, 1.0f);
    Vector3 mid_vector = vec3_Slerp(&normal, &incoming_reflected, mat_reflect);
    vec3_Normalize(&mid_vector);
    Matrix3 rot_mat;
    mat3_CreateRotationMatrix(&rot_mat, &hemi_normal, &mid_vector); //&incoming_reflected
    Vector3 new_dir = mat3_MultiplyMatrixAndVector(&rot_mat, &unit_rand_dir);
    *probability = (1.0f - (1.0f / (2.0f * PI))) * mat_reflect + (1.0f / (2.0f * PI));
    // Clamp the new direction into the hemisphere. If the new direction is
    // below the hemisphere, we rotate it until it is back in range.
    float normal_dot = vec3_DotProduct(&new_dir, &normal);
    if(normal_dot < 0.0f){
        new_dir = vec3_Rotate(&mid_vector, &new_dir, PI);
    }
    return new_dir;
}

Vector3 light_GetDirectionInHemisphere(Intersection* intsec, float* probability){
    Vector3 normal = intsec_GetIntersectNormal(intsec);
    // Choose a random direction in a hemisphere.
    Vector3 unit_rand_dir;
    float u1 = (float)rand() / (float)RAND_MAX;
    float u2 = (float)rand() / (float)RAND_MAX;
    float rad = sqrtf(u1);
    float theta = 2.0f * PI * u2;
    float x = rad * cosf(theta);
    float y = rad * sinf(theta);
    float z = sqrtf(1.0f - u1);
    if(1.0f - u1 < 0.0f){
        z = sqrtf(0.0f);
    }
    vec3_Create(&unit_rand_dir, x, y, z);
    // Rotate "unit_rand_dir" into the same basis as the normal vector.
    Vector3 hemi_normal;
    vec3_Create(&hemi_normal, 0.0f, 0.0f, -1.0f);
    Matrix3 rot_mat;
    mat3_CreateRotationMatrix(&rot_mat, &normal, &hemi_normal);
    Vector3 new_dir = mat3_MultiplyMatrixAndVector(&rot_mat, &unit_rand_dir);

    // Calculate the probability of this new ray.
    *probability = 1.0f / (2.0f * PI);
    return new_dir;
}

Color light_GetDirectLightContribution(Light* light, Intersection* source_intsec, Vector3* cam_loc){
    Color contribution;
    col_CreateGrayscale(&contribution, 0.0f);
    Color diffuse = col_CreateCopy(&(light->color));
    Color specular = col_CreateCopy(&(light->color));

    Vector3 collision_pos = intsec_GetIntersectPosition(source_intsec);
    Vector3 collision_normal = intsec_GetIntersectNormal(source_intsec);

    Vector3 to_light_dir = vec3_CreateCopy(&(light->origin));
    vec3_SubVectors(&to_light_dir, &collision_pos);
    float r2 = vec3_LengthSquared(&to_light_dir);
    vec3_Normalize(&to_light_dir);

    // Calculate the light intensity
    float intensity = light->power * 50.0f / (4.0f * PI * r2);

    // Diffuse shading using Lambert
    float cos_theta = vec3_DotProduct(&collision_normal, &to_light_dir);
    if(cos_theta < 0.0f){
        cos_theta = 0.0f;
    }
    float diffuse_coeff = (*source_intsec).mat.diffuse;
    col_MulColorByValue(&diffuse, (cos_theta * intensity * diffuse_coeff));

    // Specular shading using Blinn-Phong
    Vector3 to_cam = vec3_CreateCopy(cam_loc);
    vec3_SubVectors(&to_cam, &collision_pos);
    vec3_Normalize(&to_cam);
    vec3_AddVectors(&to_cam, &to_light_dir);
    vec3_Normalize(&to_cam);
    float reflect_cos_theta = vec3_DotProduct(&collision_normal, &to_cam);
    if(reflect_cos_theta < 0.0f){
        reflect_cos_theta = 0.0f;
    }
    float mat_reflect = (*source_intsec).mat.reflectance;
    float specular_value = powf(reflect_cos_theta, mat_reflect*mat_reflect*1000.0f) * intensity;
    float specular_coeff = (*source_intsec).mat.specular;
    col_MulColorByValue(&specular, (specular_value * specular_coeff));

    // Sum the diffuse and specular components
    col_AddColors(&contribution, &diffuse);
    col_AddColors(&contribution, &specular);
    return contribution;
}

//
// MATERIALS
//

int mat_Create(Material* new_mat, Color* alb, float diff, float spec, float amb, float ref){
    new_mat->albedo = col_CreateCopy(alb);
    new_mat->specular = spec;
    new_mat->diffuse = diff;
    new_mat->ambient = amb;
    new_mat->reflectance = ref;
    return 1;
}

Material mat_CreateCopy(Material* mat){
    Material new_mat;
    new_mat.albedo = col_CreateCopy(&(mat->albedo));
    new_mat.specular = mat->specular;
    new_mat.diffuse = mat->diffuse;
    new_mat.ambient = mat->ambient;
    new_mat.reflectance = mat->reflectance;
    return new_mat;
}

//
// SHAPES
//

Shape shape_CreateCopy(Shape* shape){
    Shape new_shape;
    new_shape.shape_type = shape->shape_type;
    new_shape.pos = vec3_CreateCopy(&(shape->pos));
    new_shape.plane_normal = vec3_CreateCopy(&(shape->plane_normal));
    new_shape.sphere_radius = shape->sphere_radius;
    new_shape.mat = mat_CreateCopy(&(shape->mat));
    return new_shape;
}

int shape_CreateEmpty(Shape* shape){
    shape->shape_type = EMPTY;

    Vector3 empty_vec;
    vec3_CreateSingle(&empty_vec, 0.0f);
    Color empty_col;
    col_CreateGrayscale(&empty_col, 0.0f);
    Material empty_mat;
    mat_Create(&empty_mat, &empty_col, 0.0f, 0.0f, 0.0f, 0.0f);
    shape->pos = empty_vec;
    shape->mat = empty_mat;

    shape->plane_normal = empty_vec;
    shape->sphere_radius = 0.0f;
    return 1;
}

int shape_CreatePlane(Shape* shape, Material* mat, Vector3* pos, Vector3* normal){
    shape->pos = vec3_CreateCopy(pos);
    shape->mat = mat_CreateCopy(mat);
    shape->plane_normal = vec3_CreateCopy(normal);
    shape->shape_type = PLANE;

    shape->sphere_radius = 0.0f;
    return 1;
}

int shape_CreateSphere(Shape* shape, Material* mat, Vector3* pos, float radius){
    shape->pos = vec3_CreateCopy(pos);
    shape->mat = mat_CreateCopy(mat);
    shape->sphere_radius = radius;
    shape->shape_type = SPHERE;

    Vector3 empty_vec;
    vec3_Create(&empty_vec, 0.0f, 0.0f, 0.0f);
    shape->plane_normal = empty_vec;
    return 1;
}

int shape_FindPlaneIntersection(Shape* plane, Intersection* result_intsec){
    Vector3 intsec_dir = (*result_intsec).ray.dir;
    Vector3 intsec_origin = (*result_intsec).ray.origin;
    float d_dot_n = vec3_DotProduct(&intsec_dir, &(plane->plane_normal));
    if(d_dot_n == 0.0f){
        return 0;
    }
    Vector3 plane_pos_copy = vec3_CreateCopy(&(plane->pos));
    vec3_SubVectors(&plane_pos_copy, &intsec_origin);
    float t = vec3_DotProduct(&plane_pos_copy, &(plane->plane_normal)) / d_dot_n;
    if(t <= RAY_DIST_MIN || t >= result_intsec->t){     // Outside of range?
        return 0;
    }
    result_intsec->t = t;
    result_intsec->shape = shape_CreateCopy(plane);
    result_intsec->mat = plane->mat;
    return 1;
}

int shape_FindSphereIntersection(Shape* sphere, Intersection* result_intsec){
    Ray local_ray = ray_CreateCopy(&(result_intsec->ray));
    vec3_SubVectors(&(local_ray.origin), &(sphere->pos));

    float a = vec3_LengthSquared(&(local_ray.dir));
    float b = 2.0f * vec3_DotProduct(&(local_ray.dir), &(local_ray.origin));
    float c = vec3_LengthSquared(&(local_ray.origin)) - powf(sphere->sphere_radius, 2.0f);

    float discriminant = powf(b, 2.0f) - (4.0f * a * c);
    if(discriminant < 0.0f){
        return 0;
    }

    float t1 = (-b - sqrtf(discriminant)) / (2.0f * a);
    float t2 = (-b + sqrtf(discriminant)) / (2.0f * a);

    if(t1 > RAY_DIST_MIN && t1 < result_intsec->t){
        result_intsec->t = t1;
    } else if(t2 > RAY_DIST_MIN && t2 < result_intsec->t){
        result_intsec->t = t2;
    } else {
        return 0;
    }
    result_intsec->shape = shape_CreateCopy(sphere);
    result_intsec->mat = sphere->mat;
    return 1;
}

int shape_FindIntersection(Shape* shape, Intersection* result_intsec){
    int result = 0;
    switch(shape->shape_type){
        case PLANE: 
            result = shape_FindPlaneIntersection(shape, result_intsec);
            break;
        case SPHERE: 
            result = shape_FindSphereIntersection(shape, result_intsec);
            break;
        case EMPTY:
            printf("WARNING: Empty shape!\n");
            return 0;
    }
    return result;
}

int shape_DoesIntersect(Shape* shape, Ray* ray){
    int result = 0;
    switch(shape->shape_type){
        case PLANE: 
            result = shape_DoesPlaneIntersect(shape, ray);
            break;
        case SPHERE: 
            result = shape_DoesSphereIntersect(shape, ray);
            break;
        case EMPTY:
            printf("WARNING: Empty shape!\n");
            return 0;
    }
    return result;
}

int shape_DoesPlaneIntersect(Shape* plane, Ray* ray){
    Vector3 ray_dir = ray->dir;
    Vector3 ray_origin = ray->origin;
    float d_dot_n = vec3_DotProduct(&ray_dir, &(plane->plane_normal));
    if(d_dot_n == 0.0f){
        return 0;
    }
    Vector3 plane_pos_copy = vec3_CreateCopy(&(plane->pos));
    vec3_SubVectors(&plane_pos_copy, &ray_origin);
    float t = vec3_DotProduct(&plane_pos_copy, &(plane->plane_normal)) / d_dot_n;
    if(t <= RAY_DIST_MIN || t >= ray->t_max){     // Outside of range?
        return 0;
    }
    return 1;
}

int shape_DoesSphereIntersect(Shape* sphere, Ray* ray){
    Ray local_ray = ray_CreateCopy(ray);
    vec3_SubVectors(&(local_ray.origin), &(sphere->pos));

    float a = vec3_LengthSquared(&(local_ray.dir));
    float b = 2 * vec3_DotProduct(&(local_ray.dir), &(local_ray.origin));
    float c = vec3_LengthSquared(&(local_ray.origin)) - powf(sphere->sphere_radius, 2.0f);

    float discriminant = powf(b, 2.0f) - (4.0f * a * c);
    if(discriminant < 0.0f){
        return 0;
    }

    float t1 = (-b - sqrtf(discriminant)) / (2.0f * a);
    float t2 = (-b + sqrtf(discriminant)) / (2.0f * a);

    if(t1 > RAY_DIST_MIN && t1 < ray->t_max){
        return 1;
    } else if(t2 > RAY_DIST_MIN && t2 < ray->t_max){
        return 1;
    } else {
        return 0;
    }
}

//
// SCENE
//

int scene_Create(Scene* new_scene){
    new_scene->set_shapes_size = 0;
    new_scene->set_lights_size = 0;
    return 1;
}

Scene scene_CreateCopy(Scene* scene){
    Scene new_scene;
    memcpy(&(new_scene.shapes), &(scene->shapes), sizeof(Shape)*SHAPE_MAX);
    new_scene.set_shapes_size = scene->set_shapes_size;
    return new_scene;
}

int scene_AddShape(Scene* scene, Shape* shape){
    Shape* shape_array = scene->shapes;
    if(scene->set_shapes_size < SHAPE_MAX){
        shape_array[scene->set_shapes_size] = shape_CreateCopy(shape);
        scene->set_shapes_size = scene->set_shapes_size + 1;
        return 1;
    } else {
        return 0;
    }
}

int scene_AddLight(Scene* scene, Light* light){
    Light* light_array = scene->lights;
    if(scene->set_lights_size < LIGHT_MAX){
        light_array[scene->set_lights_size] = light_CreateCopy(light);
        scene->set_lights_size = scene->set_lights_size + 1;
        return 1;
    } else {
        return 0;
    }
}

int scene_FindIntersections(Scene* scene, Intersection* result_intsec){
    int does_intersect = 0;
    Shape* shape_array = scene->shapes;

    for(int i = 0; i < scene->set_shapes_size; i++){
        Shape curr_shape = shape_array[i];
        int result = shape_FindIntersection(&curr_shape, result_intsec);
        if(result){
            does_intersect = 1;
        }
    }
    return does_intersect;
}

int scene_HasIntersection(Scene* scene, Ray* ray){
    Shape* shape_array = scene->shapes;

    for(int i = 0; i < scene->set_shapes_size; i++){
        Shape curr_shape = shape_array[i];
        int result = shape_DoesIntersect(&curr_shape, ray);
        if(result){
            return 1;
        }
    }
    return 0;
}

int scene_HasShadowIntersection(Scene* scene, Light* light, Intersection* source){
    Vector3 collision_pos = intsec_GetIntersectPosition(source);
    //Vector3 collision_normal = intsec_GetIntersectNormal(source);
    Vector3 to_light_dir = vec3_CreateCopy(&(light->origin));
    vec3_SubVectors(&to_light_dir, &collision_pos);
    float light_dist = vec3_Normalize(&to_light_dir);

    Ray shadow_ray;
    ray_Create(&shadow_ray, &collision_pos, &to_light_dir, light_dist);
    return scene_HasIntersection(scene, &shadow_ray);
}

//
// INTERSECTIONS
//

int intsec_Create(Intersection* new_intsec, Ray* ray){
    new_intsec->ray = ray_CreateCopy(ray);
    new_intsec->t = ray->t_max;
    Shape empty;
    shape_CreateEmpty(&empty);
    new_intsec->shape = empty;
    return 1;
}

Intersection intsec_CreateCopy(Intersection* intsec){
    Intersection new_intsec;
    new_intsec.ray = intsec->ray;
    new_intsec.shape = intsec->shape;
    new_intsec.t = intsec->t;
    new_intsec.mat = intsec->mat;
    return new_intsec;
}

int intsec_HasIntersected(Intersection* intsec){
    int shape_type = (intsec->shape).shape_type;
    return (shape_type != EMPTY);
}

Vector3 intsec_GetIntersectPosition(Intersection* intsec){
    return ray_GetPointOnRay(&(intsec->ray), intsec->t);
}

Vector3 intsec_GetIntersectNormal(Intersection* intsec){
    Vector3 normal;
    Shape intsec_shape = intsec->shape;
    Vector3 intsec_point = intsec_GetIntersectPosition(intsec);
    switch(intsec_shape.shape_type){      // Get the normal first.
        case PLANE:
            normal = vec3_CreateCopy(&(intsec_shape.plane_normal));
            break;
        case SPHERE:
            normal = vec3_CreateCopy(&intsec_point);
            vec3_SubVectors(&normal, &(intsec_shape.pos));
            vec3_Normalize(&normal);
            break;
        case EMPTY:
            printf(" WARNING: Empty shape!");
            vec3_Create(&normal, 0.0f, 1.0f, 0.0f);
            break;
    }
    return normal;
}

//
// CAMERA
//

int cam_Create(Camera* new_cam, Vector3* origin, Vector3* target, Vector3* up, float fov, float asp_ratio){
    Vector3 new_forward = vec3_CreateCopy(target);
    vec3_SubVectors(&new_forward, origin);
    vec3_Normalize(&new_forward);

    Vector3 new_right = vec3_CrossProduct(&new_forward, up);
    vec3_Normalize(&new_right);

    Vector3 new_up = vec3_CrossProduct(&new_right, &new_forward);
    //vec3_Normalize(&new_up);

    new_cam->origin = vec3_CreateCopy(origin);
    new_cam->forward = new_forward;
    new_cam->right = new_right;
    new_cam->up = new_up;
    new_cam->height = tanf(fov);
    new_cam->width = new_cam->height * asp_ratio;

    return 1;
}

Camera cam_CreateCopy(Camera* cam){
    Camera new_cam;
    new_cam.origin = cam->origin;
    new_cam.forward = cam->forward;
    new_cam.right = cam->right;
    new_cam.up = cam->up;
    new_cam.width = cam->width;
    new_cam.height = cam->height;
    return new_cam;
}

Ray cam_MakeRay(Camera* cam, Vector2* screen_point){
    Vector3 dir_xcomp = vec3_CreateCopy(&(cam->right));
    vec3_MulVector(&dir_xcomp, (screen_point->u * cam->width));
    Vector3 dir_ycomp = vec3_CreateCopy(&(cam->up));
    vec3_MulVector(&dir_ycomp, (screen_point->v * cam->height));
    Vector3 dir_total = vec3_CreateCopy(&(cam->forward));
    vec3_AddVectors(&dir_total, &dir_xcomp);
    vec3_AddVectors(&dir_total, &dir_ycomp);
    vec3_Normalize(&dir_total);

    Ray final_ray;
    ray_Create(&final_ray, &(cam->origin), &dir_total, RAY_DIST_MAX);
    return final_ray;
}

//
// COLOR
//

int col_ClampColor(Color* col){
    float final_r = col->r;
    float final_g = col->g;
    float final_b = col->b;
    if(col->r < 0.0f){
        final_r = 0.0f;
    }
    if(col->r > 1.0f){
        final_r = 1.0f;
    }
    if(col->g < 0.0f){
        final_g = 0.0f;
    }
    if(col->g > 1.0f){
        final_g = 1.0f;
    }
    if(col->b < 0.0f){
        final_b = 0.0f;
    }
    if(col->b > 1.0f){
        final_b = 1.0f;
    }
    col->r = final_r;
    col->g = final_g;
    col->b = final_b;
    return 1;
}

int col_Create(Color* new_col, float r, float g, float b){
    new_col->r = r;
    new_col->g = g;
    new_col->b = b;
    return 1;
}

int col_CreateGrayscale(Color* new_col, float s){
    new_col->r = s;
    new_col->g = s;
    new_col->b = s;
    return 1;
}

Color col_CreateCopy(Color* col){
    Color new_col;
    new_col.r = col->r;
    new_col.g = col->g;
    new_col.b = col->b;
    return new_col;
}

int col_AddColors(Color* col_a, Color* col_b){
    col_a->r = col_a->r + col_b->r;
    col_a->g = col_a->g + col_b->g;
    col_a->b = col_a->b + col_b->b;
    return 1;
}

int col_MulColors(Color* col_a, Color* col_b){
    col_a->r = col_a->r * col_b->r;
    col_a->g = col_a->g * col_b->g;
    col_a->b = col_a->b * col_b->b;
    return 1;
}

int col_MulColorByValue(Color* col, float val){
    col->r = col->r * val;
    col->g = col->g * val;
    col->b = col->b * val;
    return 1;
}

int col_ApplyGammaCorrection(Color* col, float exposure, float gamma){
    col->r = powf(col->r * exposure, 1.0f/gamma);
    col->g = powf(col->g * exposure, 1.0f/gamma);
    col->b = powf(col->b * exposure, 1.0f/gamma);
    return 1;
}

int col_GetColorDataLength(Color* col){
    int r_adj = (int)(col->r * 255.0f);
    int g_adj = (int)(col->g * 255.0f);
    int b_adj = (int)(col->b * 255.0f);
    int r_len = snprintf(NULL, 0, "%3d ", r_adj);
    int g_len = snprintf(NULL, 0, "%3d ", g_adj);
    int b_len = snprintf(NULL, 0, "%3d\0", b_adj);
    return r_len + g_len + b_len + 1;
}

int col_ReadColor(char* dest_str, Color* col){
    int r_adj = (int)(col->r * 255.0f);
    int g_adj = (int)(col->g * 255.0f);
    int b_adj = (int)(col->b * 255.0f);
    int r_len = snprintf(NULL, 0, "%3d ", r_adj);
    int g_len = snprintf(NULL, 0, "%3d ", g_adj);
    int b_len = snprintf(NULL, 0, "%3d\0", b_adj);
    char final_str[r_len + g_len + b_len + 1];
    char r_comp[r_len];
    sprintf(r_comp, "%3d ", r_adj);
    strncat(final_str, r_comp, r_len);
    char g_comp[g_len];
    sprintf(g_comp, "%3d ", g_adj);
    strncat(final_str, g_comp, g_len);
    char b_comp[b_len];
    sprintf(b_comp, "%3d\0", b_adj);
    strncat(final_str, b_comp, b_len);
    strcpy(dest_str, final_str);
    return 1;
}

//
// IMAGE
//

int img_Create(Image* img, int w, int h){
    img->width = w;
    img->height = h;
    img->data_size = w*h;
    img->data = (Color*) malloc(sizeof(Color) * img->data_size);
    if(img->data == NULL){
        return 0;
    }
    return 1;
}

int img_GetWidth(Image* img){
    return img->width;
}

int img_GetHeight(Image* img){
    return img->height;
}

Color* img_GetPixel(Image* img, int x, int y){
    Color* pixel;
    int row = y * img_GetWidth(img);
    if(row + x < img->data_size){
        pixel = &((img->data)[row + x]);
    } else {
        pixel = NULL;
    }
    return pixel;
}

int img_SetPixel(Color* pixel, Color val){
    pixel->r = val.r;
    pixel->g = val.g;
    pixel->b = val.b;
    return 1;
}

Vector2 img_GetPixelCoord(Image* img, int arr_pos){
    Vector2 coords;
    int img_width = img_GetWidth(img);
    int y = (int)(arr_pos / img_width);
    int x = arr_pos - (y * img_width);
    vec2_Create(&coords, x, y);
    return coords;
}

int img_SaveImage(Image* img, const char* dir){   //PPM Format
    int img_width = img_GetWidth(img);
    int img_height = img_GetHeight(img);

    FILE* f = fopen(dir, "w");
    if(f == NULL){
        printf("Error opening the file: %s\n", dir);
        return 0;
    }
    fprintf(f, "P3\n");     // .PPM Header
    fprintf(f, "%d %d\n", img_width, img_height);
    fprintf(f, "%d\n", 255);
    for(int y = 0; y < img_height; y++){        // Save rows second.
        for(int x = 0; x < img_width; x++){     // Save columns first.
            Color* col = img_GetPixel(img, x, y);
            col_ClampColor(col);
            col_ApplyGammaCorrection(col, EXPOSURE, GAMMA);
            int col_len = col_GetColorDataLength(col);
            char col_data[col_len];
            col_ReadColor(col_data, col);
            fprintf(f, "%s\n", col_data);
        }
    }
    fclose(f);
    return 1;
}

int img_Destroy(Image* img){
    free(img->data);
    return 1;
}

//
// MAIN
//

Color pathtrace(Ray* ray, Scene* scene, Vector3* cam_loc, int depth){
    Color final_col;
    Color direct_contrib;
    Color indirect_contrib;
    col_CreateGrayscale(&final_col, 0.0f);
    if(depth > RAY_BOUNCE_MAX){
        return final_col;
    }

    Intersection curr_intsec;
    intsec_Create(&curr_intsec, ray);
    int intsec_result = scene_FindIntersections(scene, &curr_intsec);
    if(intsec_result){
        final_col = col_CreateCopy(&(curr_intsec.mat.albedo));
        float mat_ref = sqrtf(curr_intsec.mat.reflectance);
        // Calculate Direct Light
        Light* light_list = scene->lights;
        col_Create(&direct_contrib, 0.0f, 0.0f, 0.0f);
        for(int l = 0; l < scene->set_lights_size; l++){
            Light curr_light = light_list[l];
            int is_visible = !scene_HasShadowIntersection(scene, &curr_light, &curr_intsec);
            Color contrib = light_GetDirectLightContribution(&curr_light, &curr_intsec, cam_loc);
            col_MulColorByValue(&contrib, (float)is_visible);
            col_AddColors(&direct_contrib, &contrib);
        }
        col_MulColorByValue(&direct_contrib, 1.0f/PI);

        // Calculate Indirect Light
        col_Create(&indirect_contrib, 0.0f, 0.0f, 0.0f);
        // Find the normal of the collision point.
        Vector3 normal = intsec_GetIntersectNormal(&curr_intsec);
        Vector3 intsec_point = intsec_GetIntersectPosition(&curr_intsec);
        // Sample random rays reflecting from this collision point.
        for(int s = 0; s < SAMPLES; s++){
            Ray new_dir;
            float prob;
            Vector3 random_dir = light_GetBiasedDirectionInHemisphere(&curr_intsec, &prob);
            if(prob <= 0.0f){
                continue;
            }
            ray_Create(&new_dir, &intsec_point, &random_dir, RAY_DIST_MAX);
            float cos_theta = vec3_DotProduct(&(new_dir.dir), &normal);
            if(cos_theta < 0.0f){
                cos_theta = 0.0f;
            }
            // Bounce ray recursively
            Color incoming_color = pathtrace(&new_dir, scene, cam_loc, depth + 1);
            col_MulColorByValue(&incoming_color, (cos_theta / prob));
            col_AddColors(&indirect_contrib, &incoming_color);
        }
        col_MulColorByValue(&indirect_contrib, (mat_ref + 1.0f)/((float)SAMPLES * PI));

        // Mix the final contributions into the original albedo and return it.
        Color final_contrib;
        col_Create(&final_contrib, 0.0f, 0.0f, 0.0f);
        col_AddColors(&final_contrib, &direct_contrib);
        col_AddColors(&final_contrib, &indirect_contrib);
        col_MulColors(&final_col, &final_contrib);
        return final_col;
    } else {
        col_CreateGrayscale(&final_col, 0.25f);
        return final_col;
    }
}

int print_progress(int curr, int total){
    double percentage_d = (double)curr * 100.0f / (double)total;
    int percentage = (int)percentage_d;
    int total_len = floor(log10(abs(total))) + 1;
    if(total == 0){
        total_len = 1;
    }
    printf("\r%.*d/%d pixels rendered on %d threads (%3d%% complete)", total_len, curr, total, THREAD_COUNT, percentage);
    fflush(stdout);
    return 1;
}

typedef struct ThreadRaytraceArgs{
    Image* img_dest;
    Camera* cam;
    Scene* scene;
    int start;
    int* progress;
    pthread_mutex_t* mutex;
    int* status_flag;
} ThreadRaytraceArgs;

void* raytrace_segment(void* thread_raytrace_args){
    // Split out all the arguments from the passing struct
    ThreadRaytraceArgs* args = (ThreadRaytraceArgs*) thread_raytrace_args;
    Image* img_dest = args->img_dest;
    Camera* cam = args->cam;
    Scene* scene = args->scene;
    int start = args->start;
    int* progress = args->progress;
    int* status_flag = args->status_flag;
    pthread_mutex_t* mutex = args->mutex;
    // Perform the raytrace.
    int img_width = img_GetWidth(img_dest);
    int img_height = img_GetHeight(img_dest);
    int total_pixels = img_width * img_height;
    for(int p = start; p < total_pixels; p += THREAD_COUNT){
        Vector2 pixel_coord = img_GetPixelCoord(img_dest, p);
        int x = (int)pixel_coord.u;
        int y = (int)pixel_coord.v;

        Vector2 screen_coord;
        float coord_x = ((2.0f * (float)x) / (float)img_width) - 1.0f;
        float coord_y = ((-2.0f * (float)y) / (float)img_height) + 1.0f;
        vec2_Create(&screen_coord, coord_x, coord_y);
        Ray curr_ray = cam_MakeRay(cam, &screen_coord);

        Color* curr_pixel = img_GetPixel(img_dest, x, y);
        Color final_color = pathtrace(&curr_ray, scene, &(cam->origin), 0);
        img_SetPixel(curr_pixel, final_color);  // Specific pixel only accessed by this thread.

        pthread_mutex_lock(mutex);
        *progress += 1;
        pthread_mutex_unlock(mutex);
    }
    //pthread_mutex_lock(mutex);
    *status_flag = 1;   // finished.
    //pthread_mutex_unlock(mutex);
    return NULL;
}

int raytrace_multithreading(Image* img_dest, Camera* cam, Scene* scene){
    // Progress counter
    int img_width = img_GetWidth(img_dest);
    int img_height = img_GetHeight(img_dest);
    int progress = 0;
    int total_pixels = img_width * img_height;

    // Set up threads and their info
    pthread_t threads[THREAD_COUNT];
    int thread_status[THREAD_COUNT];
    ThreadRaytraceArgs thread_args[THREAD_COUNT];
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    for(int t = 0; t < THREAD_COUNT; t++){
        thread_status[t] = 0;
        // Pack our arguments into the struct since pthread_create can only take one argument.
        ThreadRaytraceArgs thread_arg;
        thread_arg.img_dest = img_dest;
        thread_arg.cam = cam;
        thread_arg.scene = scene;
        thread_arg.start = t;
        thread_arg.progress = &progress;
        thread_arg.mutex = &mutex;
        thread_arg.status_flag = &(thread_status[t]);
        thread_args[t] = thread_arg;
        // Create the thread
        int err = pthread_create(&(threads[t]), NULL, raytrace_segment, &(thread_args[t]));
        if(err){
            printf("ERROR Making thread %d!\n", t);
        }
    }

    // Track progress and monitor thread status
    while(1){
        // Update the status
        print_progress(progress, total_pixels);
        // Check if there are any threads still working
        int threads_alive = THREAD_COUNT;
        for(int a = 0; a < THREAD_COUNT; a++){
            if(thread_status[a]){  // true if finished
                threads_alive--;
            }
        }
        if(!threads_alive){     // End monitoring once all threads have finished.
            break;
        }
    }
    print_progress(progress, total_pixels);
    printf("\n");

    // Join threads now that we're done
    for(int f = 0; f < THREAD_COUNT; f++){
        pthread_join(threads[f], NULL);
    }
    return 1;
}

int main(int argc, char* argv[]){
    int img_width = 800;
    int img_height = 600;
    float asp_ratio = (float)img_width / (float)img_height;
    Image final_img;
    int ok = img_Create(&final_img, img_width, img_height);
    if(!ok){
        printf("ERROR: Couldn't allocate room for image.\n");
        return 0;
    }

    Color red;
    col_Create(&red, 1.0f, 0.0f, 0.0f);
    Color blue;
    col_Create(&blue, 0.0f, 0.0f, 1.0f);
    Color green;
    col_Create(&green, 0.0f, 0.85f, 0.15f);
    Color white;
    col_Create(&white, 1.0f, 1.0f, 1.0f);
    Color light_blue;
    col_Create(&light_blue, 0.25f, 0.25f, 1.0f);
    Color light_pink;
    col_Create(&light_pink, 1.0f, 0.25f, 0.25f);
    Color gray;
    col_Create(&gray, 0.4f, 0.4f, 0.4f);

    Material light_pink_mat;
    mat_Create(&light_pink_mat, &light_pink, 0.8, 0.2, 0.05, 0.95f);
    Material light_blue_mat;
    mat_Create(&light_blue_mat, &light_blue, 0.5, 0.5, 0.05, 0.75f);
    Material reflect_gray_mat;
    mat_Create(&reflect_gray_mat, &gray, 0.2, 0.8, 0.05, 1.0f);
    Material ornament_blue_mat;
    mat_Create(&ornament_blue_mat, &blue, 0.2, 0.8, 0.05, 0.95f);
    Material diffuse_green_mat;
    mat_Create(&diffuse_green_mat, &green, 0.8, 0.2, 0.05, 0.3f);
    Material white_mat;
    mat_Create(&white_mat, &white, 0.8, 0.2, 0.05, 0.65f);

    Camera cam;
    Vector3 origin;
    vec3_Create(&origin, -5.0f, 1.0f, 0.0f);
    Vector3 target;
    vec3_Create(&target, 0.0f, 1.0f, 0.0f);
    Vector3 up;
    vec3_Create(&up, 0.0f, 1.0f, 0.0f);
    cam_Create(&cam, &origin, &target, &up, (25.0f * PI / 180.0f), asp_ratio);

    Scene scene;
    scene_Create(&scene);
    // Create some planes
    Shape plane;
    Shape wall;
    Vector3 floor;
    Vector3 wall_vec;
    Vector3 wall_pos;
    vec3_Create(&floor, 0.0f, 0.0f, 0.0f);
    vec3_Create(&wall_vec, -1.0f, 0.0f, 0.0f);
    vec3_Create(&wall_pos, 4.0f, 0.0f, 0.0f);
    shape_CreatePlane(&plane, &reflect_gray_mat, &floor, &up);
    shape_CreatePlane(&wall, &light_blue_mat, &wall_pos, &wall_vec);
    scene_AddShape(&scene, &plane);
    scene_AddShape(&scene, &wall);
    // Create a sphere
    Shape sphere;
    shape_CreateSphere(&sphere, &reflect_gray_mat, &target, 1.0f);
    scene_AddShape(&scene, &sphere);
    // Create a 2nd sphere
    Shape sphere2;
    Vector3 sphere2_loc;
    vec3_Create(&sphere2_loc, 1.0f, 1.0f, 2.0f);
    shape_CreateSphere(&sphere2, &ornament_blue_mat, &sphere2_loc, 0.75f);
    scene_AddShape(&scene, &sphere2);
    // Create a 3rd sphere
    Shape sphere3;
    Vector3 sphere3_loc;
    vec3_Create(&sphere3_loc, 1.0f, 1.0f, -2.0f);
    shape_CreateSphere(&sphere3, &diffuse_green_mat, &sphere3_loc, 0.5f);
    scene_AddShape(&scene, &sphere3);
    // Create a 4th sphere
    Shape sphere4;
    Vector3 sphere4_loc;
    vec3_Create(&sphere4_loc, -2.0f, 2.0f, 0.0f);
    shape_CreateSphere(&sphere4, &white_mat, &sphere4_loc, 0.35f);
    scene_AddShape(&scene, &sphere4);
    // Create a 5th sphere
    Shape sphere5;
    Vector3 sphere5_loc;
    vec3_Create(&sphere5_loc, -1.5f, 2.0f, -1.0f);
    shape_CreateSphere(&sphere5, &light_pink_mat, &sphere5_loc, 0.45f);
    scene_AddShape(&scene, &sphere5);
    // Create two light sources
    Light light1;
    Vector3 light1_origin;
    vec3_Create(&light1_origin, -4.0f, 3.0f, -4.0f);
    light_Create(&light1, &light1_origin, &white, 15.0f);
    scene_AddLight(&scene, &light1);
    Light light2;
    Vector3 light2_origin;
    vec3_Create(&light2_origin, -4.0f, 3.0f, 2.0f);
    light_Create(&light2, &light2_origin, &white, 15.0f);
    scene_AddLight(&scene, &light2);

    printf("Raytracing.\n");
    raytrace_multithreading(&final_img, &cam, &scene);

    printf("Saving image.\n");
    char final_dir[] = "ray_result.ppm";
    img_SaveImage(&final_img, final_dir);
    img_Destroy(&final_img);
}