#[compute]
#version 450
#extension GL_EXT_nonuniform_qualifier : require

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(set=0, binding=0) uniform sampler2D depth_buffer;
layout(set=0, binding=1) uniform sampler2D color_buffer;
layout(set=0, binding=2, rgba32f) uniform restrict writeonly image2D lidar_out;

layout(set=0, binding=3, std430) restrict buffer Params {
    vec2 resolution;    
    float seed;             // Time-based seed to animate noise      
    float noise_std_dev;    // Standard deviation for Gaussian noise
    mat4 inv_proj;          
    mat4 inv_view;        

} params;


const float PI = 3.14159265359;
// Generate uniform random numbers
float rand(vec2 co) {
    return fract(sin(dot(co, vec2(12.9898, 78.233) + params.seed)) * 43758.5453);
}

// Box-Muller Transform
float get_gaussian(vec2 co) {
    float u1 = max(rand(co), 1e-6); 
    float u2 = rand(co + 1.0);
    // We only need one component (z0) for the Z-axis noise
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2) * params.noise_std_dev;
}



void main() {
    ivec2 xy = ivec2(gl_GlobalInvocationID.xy);
    
    // 1. Correct UV Calculation
    // We add 0.5 to center the sample in the pixel for better accuracy
    vec2 screen_uv = (vec2(xy) + 0.5) / params.resolution;

    if (screen_uv.x >= 1.0 || screen_uv.y >= 1.0) {
        return;
    }

    // 2. Depth Sampling
    // Note: Godot depth is [0, 1]. NDC requires [-1, 1].
    // Most Godot projections use a reversed Z or specific range; 
    // we use the standard screen_uv * 2.0 - 1.0 for NDC.
    float depth = texture(depth_buffer, screen_uv).r;

    // 3. Intensity Calculation
    vec4 color = texture(color_buffer, screen_uv);
    float intensity = (color.r + color.g + color.b) / 3.0;

    // 4. Reconstruct Camera-Space Position
    // We use the depth to get the 3D point relative to the camera
    vec4 ndc = vec4(screen_uv.x * 2.0 - 1.0, screen_uv.y * 2.0 - 1.0, depth, 1.0);
    vec4 view_pos = params.inv_proj * ndc;
    view_pos /= view_pos.w;

    // Add Gaussian noise to point
    float raw_dist = length(view_pos.xyz);
    float noise = get_gaussian(vec2(xy));
    view_pos.xyz *= (1.0 + (noise / raw_dist));

    //To ROS2 frame
    vec3 ros_point;
    ros_point.x =  view_pos.z; // Depth becomes Forward (X)
    ros_point.y =  view_pos.x; // Godot Right becomes ROS Right (negative Y)
    ros_point.z =  view_pos.y; // Godot Up becomes ROS Up (Z)

    
    // 6. Fix the "Vertical Flip"
    // We keep this to ensure the Lidar scan doesn't look upside down 
    // compared to the camera image.
    ivec2 store_xy = ivec2(xy.x, int(params.resolution.y) - 1 - xy.y);

    imageStore(lidar_out, store_xy, vec4(ros_point, intensity));
}