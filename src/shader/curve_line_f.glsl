#version 430 core
uniform vec3 line_color;
out vec4 frag_color;

void main()
{
    frag_color=vec4(line_color,1.f);
}
