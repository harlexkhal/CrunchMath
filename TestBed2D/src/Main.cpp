#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>
#include "Geometry/Circle.h"
#include "Geometry/Box.h"

CrunchMath::Vec3 Gravity = CrunchMath::Vec3(0.0f, -9.8f, 0.0f);
CrunchMath::World gameWorld(Gravity);

std::vector<DebugRenderer::Box> Boxes;

double Current_Xpos = 0.0f;
double Current_Ypos = 0.0f;

float Window_Height = 600.0f;
float Window_Width = 800.0f;
void FrameBufferResizeCallBack(GLFWwindow* window, int width, int height)
{
    Window_Height = height;
    Window_Width = width;

    glViewport(0, 0, Window_Width, Window_Height);
}

void TrackCusorCallBack(GLFWwindow* window, double xPos, double yPos)
{
    Current_Xpos = xPos;
    Current_Ypos = yPos;
}

void MouseButtonCallBack(GLFWwindow* window, int button, int action, int mods)
{
}

void Spawn()
{  
    CrunchMath::cmBox boxshape2;
    boxshape2.Set((0.05f / 2.0f), (0.05f / 2.0f), 0.0f);

    float Xaxis = ((Current_Xpos + Current_Xpos) / Window_Width) - 1;
    float Yaxis = 1 - ((Current_Ypos + Current_Ypos) / Window_Height);

    DebugRenderer::Box newBox;
    newBox.Size = CrunchMath::Vec3(0.05f, 0.05f, 0.0f);
    newBox.Position = CrunchMath::Vec3(Xaxis, Yaxis, 0.0f);

    CrunchMath::Body* newbody = gameWorld.CreateBody(&boxshape2);
    newbody->SetPosition(newBox.Position.x, newBox.Position.y, 0.0f);
    newbody->SetOrientation(0.0f, 0.0, 0.0f, 2.0f);
    newbody->SetVelocity(0.0f, 0.0f, 0.0f);
    newbody->SetDamping(0.9f, 0.9f);
    newbody->CalculateDerivedData();
    newbody->SetMass(100);
    newbody->SetBlockInertiaTensor((newBox.Size / 2.0f), 100);
    newbody->SetAwake(true);
    newBox.body = newbody;
    Boxes.emplace_back(newBox);
}

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(Window_Width, Window_Height, "CrunchMath-TestBed2D", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, FrameBufferResizeCallBack);
    glfwSetCursorPosCallback(window, TrackCusorCallBack);
    glfwSetMouseButtonCallback(window, MouseButtonCallBack);
    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    float thetha = 0;
    int Count = 0;
    float Ts = (1.0f / 200.0f);

    /*
    * Circle Circle2;
    * Circle Circle; 
    */

    DebugRenderer::Box Box1;
    Box1.Size = CrunchMath::Vec3(100.0f, 0.5f, 0.0f);
	Box1.Position = CrunchMath::Vec3(0.0f, -0.8f, 0.0f); 

    CrunchMath::cmBox groundshape;
    groundshape.Set((Box1.Size.x / 2.0f), (Box1.Size.y / 2.0f), 0.0f);

    CrunchMath::Body* body = gameWorld.CreateBody(&groundshape);
    body->SetPosition(Box1.Position.x, Box1.Position.y, 0.0f);
    body->SetOrientation(1.0f, 0.0, 0.0f, 0.0f);
    body->SetAcceleration(CrunchMath::Vec3(0.0f, 0.0f, 0.0f));
    body->CalculateDerivedData();
    body->SetMass(0.0f);
    body->SetAwake(false);
    Box1.body = body;
    Boxes.emplace_back(Box1);
   
    float dt = 0.0f;
    float lastFrame = 0.0f;
    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        int State = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
        if (State == GLFW_PRESS)
            Spawn();

        gameWorld.Step(Ts);
        for (int a = 0; a < Boxes.size(); a++)
            Boxes[a].Render();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}