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
    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    float thetha = 0;
    int Count = 0;
    float Ts = (1.0f / 200.0f);
   
    Circle Circle2(vertexShaderSource, fragmentShaderSource);
    Circle Circle(vertexShaderSource, fragmentShaderSource);

    CrunchMath::World gameWorld(CrunchMath::Vec3(0.0f, -9.8f, 0.0f));
    Box Boxes[2];
    Box Box2(vertexShaderSource, fragmentShaderSource);
    Box Box1(vertexShaderSource, fragmentShaderSource);
    // Set the first block
    Box1.Size = CrunchMath::Vec3(0.8f, 0.05f, 0.0f);
    Box2.Size = CrunchMath::Vec3(0.09f, 0.09f, 0.0f);

    Box1.Position = CrunchMath::Vec3(-0.0f, -0.4f, 0.0f);

    CrunchMath::Box boxshape;
    boxshape.SetHalfExtent((Box1.Size.x / 2.0f), (Box1.Size.y / 2.0f), 0.0f);

    CrunchMath::Body* body = gameWorld.CreateBody(&boxshape);
    body->SetPosition(Box1.Position.x, Box1.Position.y, 0.0f);
    body->SetOrientation(1.0f, 0.0, 0.0f, 2.0f);
    body->SetVelocity(0.0f, 0.0f, 0.0f);
    body->SetDamping(0.9f, 0.9f);
    body->CalculateDerivedData();
    body->SetAcceleration(CrunchMath::Vec3(0.0f, 0.0f, 0.0f));
    body->SetMass(0.0f);
    body->SetBlockInertiaTensor((Box1.Size / 2.0f), 100.0f);
    body->SetAwake(true);
    Box1.body = body;

    CrunchMath::Box boxshape2;
    boxshape2.SetHalfExtent((Box2.Size.x / 2.0f), (Box2.Size.y / 2.0f), 0.0f);

    CrunchMath::Body* body2 = gameWorld.CreateBody(&boxshape2);
    body2->SetPosition(Box2.Position.x, Box2.Position.y, 0.0f);
    body2->SetOrientation(0.0f, 0.0, 0.0f, 2.0f);
    body2->SetVelocity(0.0f, 0.0f, 0.0f);
    body2->SetDamping(0.9f, 0.9f);
    body2->CalculateDerivedData();
    body2->SetMass(1000);
    body2->SetBlockInertiaTensor((Box2.Size / 2.0f), 1000);
    body2->SetAwake(true);
    Box2.body = body2;

    Boxes[0] = Box1;
    Boxes[1] = Box2;
 
    float dt = 0.0f;
    float lastFrame = 0.0f;
    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Set up the collision data structure
       
        gameWorld.Step(Ts);

        for (int a = 0; a < 2; a++)
        {
            Boxes[a].Model = Boxes[a].body->GetTransform();
            Boxes[a].Model.Scale(Boxes[a].Size);
            Boxes[a].Render();
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