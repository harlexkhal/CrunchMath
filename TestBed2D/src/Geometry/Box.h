#pragma once
#include "Geometry.h"

class Box : public Geometry
{
public:
    CrunchMath::Vec3 Size;
    CrunchMath::OBB Volume;
private:
    float vertices[12 * 2];

public:
    Box(const char* vertexShaderSource, const char* fragmentShaderSource)
    {
        CreateShaderProgram(vertexShaderSource, fragmentShaderSource);
        Draw();
        AllocateBuffers(sizeof(vertices), vertices);

        Size = CrunchMath::Vec3(0.5f, 0.2f, 0.0f);
        Model.SetRotate(CrunchMath::Vec3(0, 0, 1), CrunchMath::Radian(45));
        Model.Scale(Size);
        CrunchMath::Vec4 R(0, 0, 1, CrunchMath::Radian(45));
        Volume = CrunchMath::OBB(Position, R, (Size / 2));
        Color = CrunchMath::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
    }

protected:
    virtual void Draw() override
    {
        vertices[0] = -0.5f; vertices[1] = 0.5f; vertices[2] = 0.0f;
        vertices[3] = 0.5f; vertices[4] = 0.5f; vertices[5] = 0.0f;

        vertices[6] = 0.5f; vertices[7] = 0.5f; vertices[8] = 0.0f;
        vertices[9] = 0.5f; vertices[10] = -0.5f; vertices[11] = 0.0f;

        vertices[12] = 0.5f; vertices[13] = -0.5f; vertices[14] = 0.0f;
        vertices[15] = -0.5f; vertices[16] = -0.5f; vertices[17] = 0.0f;

        vertices[18] = -0.5f; vertices[19] = -0.5f; vertices[20] = 0.0f;
        vertices[21] = -0.5f; vertices[22] = 0.5f; vertices[23] = 0.0f;
    }
};

