#version 430 core

uniform vec3 color;
uniform vec3 view_pos;
out vec4 frag_color;
in vec3 world_pos;
in vec3 normal;
vec3 light_direction=normalize(vec3(0.f,0.f,-1.f));
float ka=0.4f;
float kd=0.5f;
float ks=0.7f;
float shininess=64.f;


vec3 phongShading(vec3 diffuse_color,vec3 n)
{
    vec3 ambient=ka*diffuse_color;
    vec3 specular=ks*pow(max(dot(n,(normalize(view_pos-world_pos)-light_direction)/2.f),0.f),shininess)*vec3(1.f);
    vec3 diffuse=kd*max(dot(n,-light_direction),0.f)*diffuse_color;
    return ambient+specular+diffuse;
}
void main()
{
    frag_color=vec4(phongShading(color,normal),1.f);
}
