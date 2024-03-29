#pragma once
#include "Geometry.h"

namespace DebugRenderer {

	class Circle : public Geometry
	{
	public:
		float Size; //Diametre
		//CrunchMath::Sphere Volume;
	private:
		float vertices[2200];

	public:
		Circle()
		{
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

			CreateShaderProgram(vertexShaderSource, fragmentShaderSource);
			Draw();
			AllocateBuffers(sizeof(vertices), vertices);

			Size = 0.2f;
			Model.SetRotate(CrunchMath::Vec3(0, 0, 1), CrunchMath::Radian(45));
			//Volume = CrunchMath::Sphere(Position, Size/2);
			Model.Scale(Size);
			Color = CrunchMath::Vec4(0.0f, 0.0f, 1.0f, 1.0f);
		}

	protected:
		virtual void Draw() override
		{
			int Circum = 2200;
			float x = 0.0f;
			float y = 0.0f;
			vertices[0] = x;
			vertices[1] = y;
			vertices[2] = 0.0f;

			int t = 0;
			bool start = true;
			float kx = 0;
			float ky = 0;

			for (int i = 3; Circum; t++)
			{
				x = cos(CrunchMath::Radian(t)) * 0.5f;
				y = sin(CrunchMath::Radian(t)) * 0.5f;

				vertices[i++] = x;
				if (i >= Circum)
					break;
				vertices[i++] = y;
				if (i >= Circum)
					break;
				vertices[i++] = 0.0f;
				if (i >= Circum)
					break;

				vertices[i++] = x;
				if (i >= Circum)
					break;
				vertices[i++] = y;
				if (i >= Circum)
					break;
				vertices[i++] = 0.0f;
			}
		}
	};
}