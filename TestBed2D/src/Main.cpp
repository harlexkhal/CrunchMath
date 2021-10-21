#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>
#include "Geometry/Circle.h"
#include "Geometry/Box.h"


CrunchMath::Vec3 Gravity = CrunchMath::Vec3(0.0f, -9.8f, 0.0f);
CrunchMath::World gameWorld(Gravity);

std::vector<DebugRenderer::Box> Boxes;

void FrameBufferResizeCallBack(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void TrackCusorCallBack(GLFWwindow* window, double xPos, double yPos)
{
	CrunchMath::cmBox boxshape2;
	boxshape2.Set((0.05f / 2.0f), (0.05f / 2.0f), 0.0f);

	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
	{
		float Xaxis = ((xPos + xPos) / 800.0f) - 1;
		float Yaxis = 1 - ((yPos + yPos) / 600.0f);

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
}

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "CrunchMath-TestBed2D", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, FrameBufferResizeCallBack);
    glfwSetCursorPosCallback(window, TrackCusorCallBack);
    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    float thetha = 0;
    int Count = 0;
    float Ts = (1.0f / 200.0f);
   
     //Circle Circle2; 
    //Circle Circle; 


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

        gameWorld.Step(Ts);

        for (int a = 0; a < Boxes.size(); a++)
        {
            Boxes[a].Render();
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
