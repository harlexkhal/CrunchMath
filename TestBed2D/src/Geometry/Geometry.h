#pragma once
#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <CrunchMath.h>

class Geometry
{
public:
    CrunchMath::Vec4 Color;
    CrunchMath::Mat4x4 Model;
    CrunchMath::Vec3 Position;

protected:
    unsigned int ProgramID;
    unsigned int VAO;
    unsigned int VBO;
    unsigned int SizeofVertices;

public:
	Geometry(){};

    virtual void Render()
    {
        glUseProgram(ProgramID);
        glUniformMatrix4fv(glGetUniformLocation(this->ProgramID, "Model"), 1, GL_FALSE, &Model.Matrix[0][0]);
        glUniform4f(glGetUniformLocation(this->ProgramID, "Color"), Color.x, Color.y, Color.z, Color.a);
        glBindVertexArray(VAO);
        glDrawArrays(GL_LINES, 0, SizeofVertices / sizeof(float));
    }

protected:
    virtual void Draw() = 0;

	virtual void CreateShaderProgram(const char* vertexShaderSource, const char* fragmentShaderSource) 
    {
        int vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
        glCompileShader(vertexShader);
        // check for shader compile errors
        int success;
        char infoLog[512];
        glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
        }
        // fragment shader
        int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
        glCompileShader(fragmentShader);
        // check for shader compile errors
        glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
        }
       
        int shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);
       
        glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        }
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        ProgramID = shaderProgram;
	}
    
    void AllocateBuffers(unsigned int sizeofvertices, float* vertices) 
    {
        this->SizeofVertices = sizeofvertices;
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeofvertices, vertices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindVertexArray(0);
    }
};