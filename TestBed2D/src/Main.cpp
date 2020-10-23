#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Geometry/Circle.h"
#include "Geometry/Box.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

const char* vertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec3 aPos;\n"
"uniform mat4 Model;\n"
"void main()\n"
"{\n"
"   gl_Position = Model * vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
"}\0";

const char* fragmentShaderSource = "#version 330 core\n"
"out vec4 FragColor;\n"
"uniform vec4 Color;\n"
"void main()\n"
"{\n"
"   FragColor = Color;\n"
"}\n\0";

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "CrunchMath", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    float thetha = 0;
    int Count = 0;
    float FixedTimeStep = (1 / 100.0f);

    Circle Circle2(vertexShaderSource, fragmentShaderSource);
    Circle Circle(vertexShaderSource, fragmentShaderSource);
   
    Box Box2(vertexShaderSource, fragmentShaderSource);
    Box Box(vertexShaderSource, fragmentShaderSource);

    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        //Circle
        Circle.Model.Rotate(CrunchMath::Vec3(0, 0, 1), CrunchMath::WrapPi(thetha));
        Circle.Volume.Set(Box.Position, Circle.Size / 2);
        Circle.Model.Translate(Box.Position);
        Circle.Model.Scale(Circle.Size);
        Circle.Render();

        //Circle2
        Circle2.Model.Rotate(CrunchMath::Vec3(0, 0, 1), CrunchMath::WrapPi(thetha));
        Circle2.Volume.Set(Box2.Position, Circle2.Size / 2);
        Circle2.Model.Translate(Box2.Position);
        Circle2.Model.Scale(Circle2.Size);
        Circle2.Render();
        
        if (Circle.Volume.IntersectTest(Circle2.Volume))
        {
            Circle.Color = CrunchMath::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
            Circle2.Color = CrunchMath::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
        }
        else
        {
            Circle.Color = CrunchMath::Vec4(1.0f, 1.0f, 0.0f, 1.0f);
            Circle2.Color = CrunchMath::Vec4(1.0f, 1.0f, 0.0f, 1.0f);
        }

        //Box
        CrunchMath::Vec4 R(0, 0, 1, -CrunchMath::WrapPi(thetha));
        Box.Volume.Set(Box.Position, R, (Box.Size / 2));
        Box.Model.Rotate(CrunchMath::Vec3(0, 0, 1), -CrunchMath::WrapPi(thetha));
        Box.Model.Translate(Box.Position);
        Box.Model.Scale(Box.Size);
        Box.Render();
       
        //Box2
        Box2.Volume.Set(Box2.Position, R, (Box2.Size / 2));
        Box2.Model.Rotate(CrunchMath::Vec3(0, 0, 1), -CrunchMath::WrapPi(thetha));
        Box2.Model.Translate(Box2.Position);
        Box2.Model.Scale(Box2.Size);
        Box2.Render();

        //Events
        thetha = CrunchMath::Radian(Count * FixedTimeStep);
        Count += 30;

        if (glfwGetKey(window, GLFW_KEY_UP))
            Box.Position.y += 0.3f * FixedTimeStep;
        if (glfwGetKey(window, GLFW_KEY_DOWN))
            Box.Position.y -= 0.3f * FixedTimeStep;
        if (glfwGetKey(window, GLFW_KEY_LEFT))
            Box.Position.x -= 0.3f * FixedTimeStep;
        if (glfwGetKey(window, GLFW_KEY_RIGHT))
            Box.Position.x += 0.3f * FixedTimeStep;

        if (Box.Volume.IntersectTest(Box2.Volume))
        {
            Box.Color = CrunchMath::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
            Box2.Color = CrunchMath::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
        }
        else
        {
            Box.Color = CrunchMath::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
            Box2.Color = CrunchMath::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}