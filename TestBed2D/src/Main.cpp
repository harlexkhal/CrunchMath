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

    /** Holds the maximum number of Contacts. */
    const static unsigned MaxContacts = 5000;

    /** Holds the array of Contacts. */
    CrunchMath::Contact Contacts[MaxContacts];

    /** Holds the collision data structure for collision detection. */
    CrunchMath::CollisionData CData;

    /** Holds the contact Resolver. */
    CrunchMath::ContactResolver Resolver(MaxContacts);

    CData.ContactArray = Contacts;

    Circle Circle2(vertexShaderSource, fragmentShaderSource);
    Circle Circle(vertexShaderSource, fragmentShaderSource);

    Box Boxes[50];
    Box Box2(vertexShaderSource, fragmentShaderSource);
    Box Box1(vertexShaderSource, fragmentShaderSource);
    // Set the first block
    Box1.Size = CrunchMath::Vec3(0.8f, 0.05f, 0.0f);
    Box1.Position = CrunchMath::Vec3(-0.0f, -0.4f, 0.0f);
    Box1.HalfSize = CrunchMath::Vec3((Box1.Size.x / 2.0f), (Box1.Size.y / 2.0f), 0.0f);
    Box1.body->SetPosition(Box1.Position.x, Box1.Position.y, 0.0f);
    Box1.body->SetOrientation(1.0f, 0.0, 0.0f, 2.0f);
    //Box1.body->SetBlockInertiaTensor(Box1.HalfSize, 100.0f);
    Box1.body->SetVelocity(0, 0, 0);
    Box1.body->SetDamping(0.9f, 0.9f);
    Box1.body->CalculateDerivedData();
    Box1.body->SetAcceleration(CrunchMath::Vec3(0.0f, 0.0f, 0.0f));
    Box1.body->SetAwake(false);

    // Set the first block
    Box2.Size = CrunchMath::Vec3(0.05f, 0.5f, 0.0f);
    Box2.HalfSize = CrunchMath::Vec3((Box2.Size.x / 2.0f), (Box2.Size.y / 2.0f), 0.0f);
    Box2.body->SetPosition(0.1f, 0.0f, 0.0f);
    Box2.body->SetRotation(0.0f, 0.0f, 0.0f);
    Box2.body->SetOrientation(1, 0, 0, 0);
    Box2.body->SetVelocity(0, 0, 0);
    Box2.body->SetBlockInertiaTensor(Box2.HalfSize, 1000.0f);
    Box2.body->SetDamping(0.9f, 0.9f);
    Box2.body->CalculateDerivedData();
    Box2.body->SetAwake(true);

    Boxes[0] = Box1;
    Boxes[1] = Box2;

    srand((unsigned)time(NULL));
    for (int i = 0; i < 25; i++)
    {
        Box B(vertexShaderSource, fragmentShaderSource);
        // Set the first block
        B.Size = CrunchMath::Vec3(0.05f, 0.05f, 0.0f);
        B.HalfSize = CrunchMath::Vec3((B.Size.x / 2.0f), (B.Size.y / 2.0f), 0.0f);
        B.body->SetPosition((float)rand() / RAND_MAX, 0.9f, 0.0f);
        B.body->SetOrientation(1, 0, 0, 0);
        B.body->SetVelocity(0, 0, 0);
        B.body->SetRotation(0, 0, 0);
        B.body->SetMass(500.0f);
        B.body->SetBlockInertiaTensor(B.HalfSize, 500.0f);
        B.body->SetDamping(0.9f, 0.9f);
        B.body->CalculateDerivedData();
        B.body->SetAcceleration(CrunchMath::Vec3(0.0f, -9.8f, 0.0f));
        B.body->SetAwake(true);

        Boxes[i + 2] = B;
    }

    for (int i = 0; i < 25; i++)
    {
        Box B(vertexShaderSource, fragmentShaderSource);
        // Set the first block
        B.Size = CrunchMath::Vec3(0.05f, 0.05f, 0.0f);
        B.HalfSize = CrunchMath::Vec3((B.Size.x / 2.0f), (B.Size.y / 2.0f), 0.0f);
        B.body->SetPosition((float)rand() / RAND_MAX, 0.9f, 0.0f);
        B.body->SetOrientation(1, 0, 0, 0);
        B.body->SetVelocity(0, 0, 0);
        B.body->SetRotation(0, 0, 0);
        B.body->SetMass(500.0f);
        B.body->SetBlockInertiaTensor(B.HalfSize, 500.0f);
        B.body->SetDamping(0.9f, 0.9f);
        B.body->CalculateDerivedData();
        B.body->SetAcceleration(CrunchMath::Vec3(0.0f, -9.8f, 0.0f));
        B.body->SetAwake(true);

        Boxes[i + 25] = B;
    }

    float dt = 0.0f;
    float lastFrame = 0.0f;
    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Set up the collision data structure
        CData.Reset(MaxContacts);
        CData.Friction = 0.9f;
        CData.Restitution = 0.1f;
        CData.Tolerance = 0.2f;

        for (int i = 0; i < 50; i++)
        {
            Boxes[i].body->Integrate(Ts);
        }

        for (int i = 0; i < 50; i++)
        {
            for (int j = i + 1; j < 50; j++)
            {
                CrunchMath::CollisionDetector::BoxBox(Boxes[i], Boxes[j], &CData);
            }
        }

        Resolver.ResolveContacts(CData.ContactArray, CData.ContactCount, Ts);

        for (int i = 0; i < 50; i++)
        {
            if (Boxes[i].body->GetPosition().y <= -0.9f)
            {
                Boxes[i].body->SetVelocity(0, 0, 0);
                Boxes[i].body->SetRotation(0, 0, 0);
                Boxes[i].body->SetPosition(0.0f, 0.9f, 0.0f);
            }              
        }

        for (int a = 0; a < 50; a++)
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