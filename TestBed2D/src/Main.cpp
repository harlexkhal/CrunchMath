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
    float FixedTimeStep = (1.0f / 200.0f);

    /** Holds the maximum number of Contacts. */
    const static unsigned maxContacts = 1000;

    /** Holds the array of Contacts. */
    CrunchPhysx::Contact Contacts[maxContacts];

    /** Holds the collision data structure for collision detection. */
    CrunchPhysx::CollisionData cData;

    /** Holds the contact resolver. */
    CrunchPhysx::ContactResolver resolver(maxContacts);

    cData.contactArray = Contacts;

    Circle Circle2(vertexShaderSource, fragmentShaderSource);
    Circle Circle(vertexShaderSource, fragmentShaderSource);

    Box Boxes[50];
    Box Box2(vertexShaderSource, fragmentShaderSource);
    Box Box1(vertexShaderSource, fragmentShaderSource);
    // Set the first block
    Box1.Size = CrunchMath::Vec3(0.8, 0.05, 0.0f);
    Box1.Position = CrunchMath::Vec3(-0.0f, -0.3f, 0.0f);
    Box1.halfSize = CrunchPhysx::Vector3((Box1.Size.x / 2.0f), (Box1.Size.y / 2.0f), 0.0f);
    Box1.body->setPosition(Box1.Position.x, Box1.Position.y, 0.0f);
    Box1.body->setOrientation(0.5f, 0.0, 0.0f, 1.3f);
    Box1.body->setVelocity(0, 0, 0);
    Box1.body->setRotation(0, 0, 0);
    CrunchPhysx::Matrix3 it;
    Box1.body->setDamping(0.9f, 0.9f);
    Box1.body->calculateDerivedData();
    //Box1.calculateInternals();
    Box1.body->setAcceleration(CrunchPhysx::Vector3(0.0f, 0.0f, 0.0f));
    Box1.body->setAwake(true);
    Box1.body->setCanSleep(true);

    // Set the first block
    Box2.Size = CrunchMath::Vec3(0.05, 0.5, 0.0f);
    Box2.halfSize = CrunchPhysx::Vector3((Box2.Size.x / 2.0f), (Box2.Size.y / 2.0f), 0.0f);
    Box2.body->setPosition(0.1f, 0.0f, 0.0f);
    Box2.body->setOrientation(1, 0, 0, 0);
    Box2.body->setVelocity(0, 0, 0);
    Box2.body->setRotation(0, 0, 0);
    CrunchPhysx::Matrix3 it2;
    it2.setBlockInertiaTensor(Box2.halfSize, 10.0f);
    Box2.body->setInertiaTensor(it2);
    Box2.body->setDamping(0.9f, 0.9f);
    Box2.body->calculateDerivedData();
    //Box2.calculateInternals();
    Box2.body->setAcceleration(CrunchPhysx::Vector3(0.0f, 0.0f, 0.0f));
    Box2.body->setAwake(true);
    Box2.body->setCanSleep(true);

    Boxes[0] = Box1;
    Boxes[1] = Box2;

    srand((unsigned)time(NULL));
    for (int i = 0; i < 25; i++)
    {
        Box B(vertexShaderSource, fragmentShaderSource);
        // Set the first block
        B.halfSize = CrunchPhysx::Vector3((B.Size.x / 2.0f), (B.Size.y / 2.0f), 0.0f);
        B.body->setPosition((float)rand() / RAND_MAX, 0.9f, 0.0f);
        B.body->setOrientation(1, 0, 0, 0);
        B.body->setVelocity(0, 0, 0);
        B.body->setRotation(0, 0, 0);
        B.body->setMass(10.0f);
        CrunchPhysx::Matrix3 it;
        it.setBlockInertiaTensor(B.halfSize, 10.0f);
        B.body->setInertiaTensor(it);
        B.body->setDamping(0.9f, 0.9f);
        B.body->calculateDerivedData();
        //B.calculateInternals();
        B.body->setAcceleration(CrunchPhysx::Vector3(0.0f, -9.8f, 0.0f));
        B.body->setAwake(true);
        B.body->setCanSleep(true);

        Boxes[i + 2] = B;
    }

    for (int i = 0; i < 25; i++)
    {
        Box B(vertexShaderSource, fragmentShaderSource);
        // Set the first block
        B.halfSize = CrunchPhysx::Vector3((B.Size.x / 2.0f), (B.Size.y / 2.0f), 0.0f);
        B.body->setPosition((float)rand() / RAND_MAX, 0.9f, 0.0f);
        B.body->setOrientation(1, 0, 0, 0);
        B.body->setVelocity(0, 0, 0);
        B.body->setRotation(0, 0, 0);
        B.body->setMass(10.0f);
        CrunchPhysx::Matrix3 it;
        it.setBlockInertiaTensor(B.halfSize, 10.0f);
        B.body->setInertiaTensor(it);
        B.body->setDamping(0.9f, 0.9f);
        B.body->calculateDerivedData();
        //B.calculateInternals();
        B.body->setAcceleration(CrunchPhysx::Vector3(0.0f, -9.8f, 0.0f));
        B.body->setAwake(true);
        B.body->setCanSleep(true);

        Boxes[i + 25] = B;
    }
    /*float Gravity = -9.8f;
    World PlanetTest(0, Gravity, 0);

    Body* First = PlanetTest.CreateBody(CMShape::Box);
    First->SetPosition(Box2.Position.x, Box2.Position.y, 0.0f);
    First->SetSize(Box.Size.x, Box.Size.y, Box.Size.z);
    First->SetMass(1000.0f);
    First->SetInertiaTensor();
    First->SetLinearDamping(0.9f);*/

    float dt = 0.0f;
    float lastFrame = 0.0f;
    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        /*float CurrentFrame = glfwGetTime();
        dt = CurrentFrame - lastFrame;
        lastFrame = CurrentFrame;*/

        // Set up the collision data structure
        cData.reset(maxContacts);
        cData.friction = (CrunchPhysx::cpfloat)0.9;
        cData.restitution = (CrunchPhysx::cpfloat)0.2;
        cData.tolerance = (CrunchPhysx::cpfloat)0.1;

        //PlanetTest.Step(FixedTimeStep);

        for (int i = 0; i < 50; i++)
        {
            Boxes[i].body->integrate(FixedTimeStep);
        }

        for (int i = 0; i < 50; i++)
        {
            for (int j = i + 1; j < 50; j++)
            {
                CrunchPhysx::CollisionDetector::BoxBox(Boxes[i], Boxes[j], &cData);
            }
        }

        resolver.resolveContacts(
            cData.contactArray,
            cData.contactCount,
            FixedTimeStep
        );

        for (int i = 0; i < 50; i++)
        {
            if (Boxes[i].body->getPosition().y <= -0.9f)
            {
                Boxes[i].body->setVelocity(0, 0, 0);
                Boxes[i].body->setRotation(0, 0, 0);
                Boxes[i].body->setPosition(0.0f, 0.9f, 0.0f);
            }              
        }

        for (int a = 0; a < 50; a++)
        {
            int f = 0;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    Boxes[a].Model.Matrix[j][i] = Boxes[a].body->getTransform().data[f]; f++;
                    Boxes[a].Model.Matrix[0][3] = 0.0f;
                    Boxes[a].Model.Matrix[1][3] = 0.0f;
                    Boxes[a].Model.Matrix[2][3] = 0.0f;
                    Boxes[a].Model.Matrix[3][3] = 1.0f;
                }
            }

            Boxes[a].Model.Scale(Boxes[a].Size);
            Boxes[a].Render();
            //std::cout << a << std::endl;
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