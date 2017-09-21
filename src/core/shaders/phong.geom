#version 330 core

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in vec3 vWorldPosition[];
in vec3 vWorldNormal[];

out vec3 gWorldPosition; 
out vec3 gWorldNormal;

void main() 
{    
    float threshold = cos(radians(60.0));
    vec3 normal = normalize(cross(vWorldPosition[1] - vWorldPosition[0], vWorldPosition[2] - vWorldPosition[0]));

    gl_Position = gl_in[0].gl_Position;
    gWorldPosition = vWorldPosition[0];
    gWorldNormal = dot(vWorldNormal[0], normal) > threshold ? vWorldNormal[0] : normal;
    EmitVertex();

    gl_Position = gl_in[1].gl_Position;
    gWorldPosition = vWorldPosition[1];
    gWorldNormal = dot(vWorldNormal[1], normal) > threshold ? vWorldNormal[1] : normal;
    EmitVertex();

    gl_Position = gl_in[2].gl_Position;
    gWorldPosition = vWorldPosition[2];
    gWorldNormal = dot(vWorldNormal[2], normal) > threshold ? vWorldNormal[2] : normal;
    EmitVertex();
    EndPrimitive();
}  