// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

// Math
#include <math.h>  


using namespace glm;
using namespace std;
#include <common/shader.hpp>

mat2 mat24by42Mult(mat2x4 mat1, mat4x2 matrix2) {
	mat2 result;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			result[i][j] = 0;
			for (int k = 0; k < 4; k++) {
				//std::cout << matrix2[k][j] << std::endl;
				result[i][j] += mat1[i][k] * matrix2[k][j];
				if (k == 3) {
					//std::cout << result[i][j] << std::endl;
				}
			}
		}
	}
	return result;
}


mat4x2 mat42by2Mult(mat4x2 mat1, mat2 mat2) {
	mat4x2 result;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 2; j++) {
			result[i][j] = 0;
			for (int k = 0; k < 2; k++) {
				result[i][j] += mat1[i][k] * mat2[k][j];

			}
		}
	}
	return result;
}

mat4x2 pseudoinverse(mat2x4 mat) {
	mat4x2 AT = transpose(mat);
	//std::cout << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << " " << mat[0][3] << std::endl;
	//std::cout << mat[1][0] << " " << mat[1][1] << " " << mat[2][2] << " " << mat[3][3] << std::endl;
	mat2 AAT = mat24by42Mult(mat, AT);
	//std::cout << AAT[0][0] << " " << AAT[0][1] << " " << AAT[0][2] << " " << AAT[0][3] << std::endl;
	//std::cout << AAT[1][0] << " " << AAT[1][1] << " " << AAT[2][2] << " " << AAT[3][3] << std::endl;

	mat2 AATinv = inverse(AAT);
	//std::cout << AATinv[0][0] << " " << AATinv[0][1] << " " << AATinv[0][2] << " " << AATinv[0][3] << std::endl;
	//std::cout << AATinv[1][0] << " " << AATinv[1][1] << " " << AATinv[2][2] << " " << AATinv[3][3] << std::endl;
	//std::cout << AATinv[1][2] << std::endl;
	mat4x2 ans = mat42by2Mult(AT, AATinv);
	return ans;
	//return AT;
}


vec2 distance(float TargetX, float TargetY, float jointX, float jointY) {
	//std::cout << TargetY - jointY << std::endl;

	return glm::vec2(-(TargetY - jointY), TargetX - jointX);
	
}

vec4 matvecMult(mat4x2 mat, vec2 vect) {
	int rows = 4;
	vec4 result;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < rows; j++) {
			result[i] = result[i] + vect[i] * mat[i][j];
		}
	}
	return result;
}

mat4x2 GetJacobianTranspose(float posx[], float posy[], float endx, float endy) {
	//std::cout << posx[1] << std::endl; 
	vec2 Joint_A = distance(endx, endy, posx[0], posy[0]);
	//std::cout << Joint_A[0] << " " << Joint_A[1] << std::endl;
	vec2 Joint_B = distance(endx, endy, posx[1], posy[1]);
	vec2 Joint_C = distance(endx, endy, posx[2], posy[2]);
	vec2 Joint_D = distance(endx, endy, posx[3], posy[3]);
	mat4x2 matrixJ;
	/*
	std::cout << Joint_A[0] << "   " << Joint_A[1] << "   " << Joint_A[2] << std::endl;
	std::cout << Joint_B[0] << "   " << Joint_B[1] << "   " << Joint_B[2] << std::endl;
	std::cout << Joint_C[0] << "   " << Joint_C[1] << "   " << Joint_C[2] << std::endl;
	std::cout << Joint_D[0] << "   " << Joint_D[1] << "   " << Joint_D[2] << std::endl;
	*/
	matrixJ[0] = Joint_A;
	matrixJ[1] = Joint_B;
	matrixJ[2] = Joint_C;
	matrixJ[3] = Joint_D;
	mat2x4 realJ = transpose(matrixJ);
	return pseudoinverse(realJ);
}

vec4 GetDeltaOrientation(vec2 targetPos, float posx[], float posy[], float endx, float endy) {
	mat4x2 JacobianT = GetJacobianTranspose(posx, posy, endx, endy);
	vec2 vect = glm::vec2(targetPos.x - endx, targetPos.y - endy);
	//std::cout << targetPos.x - endx << "    " << targetPos.y - endy << std::endl;
	vec4 dO = matvecMult(JacobianT, vect);
	//std::cout << dO[0] << "   " << dO[1] << "   " << dO[2] << "   " << dO[3] << std::endl;
	return dO;
}

vec4 vec4Constmult(vec4 lmao, float h) {
	vec4 ans;
	for (int i = 0; i < 4; i++) {
		ans[i] = lmao[i] * h;
	}
	return ans;
}

vec4 JacobianIK(vec2 targetPos, float posx[], float posy[], float endx, float endy, float theta[]) {

	/*
	while ( (sqrt((pow((endx - targetPos.x), 2)) + (pow((endy - targetPos.y), 2)))) > 0.01) {
		vec4 dO = GetDeltaOrientation(targetPos, posx, posy, endx, endy);
		vec4 dOmulth = vec4Constmult(dO, 0.0001);
		vec4 O = glm::vec4(theta[0] + dOmulth[0], theta[1] + dOmulth[1], theta[2] + dOmulth[2], theta[3] + dOmulth[3]);
		//return O;
		return O;
	}
	*/

	//vec4 dO = glm::vec4(0, 1, 2, 3);
	vec4 dO = GetDeltaOrientation(targetPos, posx, posy, endx, endy);
	vec4 dOmulth = vec4Constmult(dO, 0.1);
	vec4 O = glm::vec4(theta[0] + dOmulth[0], theta[1] + dOmulth[1], theta[2] + dOmulth[2], theta[3] + dOmulth[3]);
	return O;
	//return glm::vec4(0, 1, 2, 3);
}

int counter = 0;
int limit = 10;

int main(void)
{
	// Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow(1024, 768, "IK sample code", NULL, NULL);
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	bool targetSet = false;
	float targetX = 0.0, targetY = 0.0;

	// Mid blue-grey background
	glClearColor(0.2f, 0.2f, 0.4f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders("TransformVertexShader.vertexshader", "ColorFragmentShader.fragmentshader");

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	// Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	//glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 220.0f);
	//For mouse interaction, it is simpler to work with an orthographic projection
	glm::mat4 Projection = glm::ortho(-60.0f, 60.0f, -60.0f, 60.0f, 0.0f, 120.0f); // In world coordinates

	// Camera matrix
	glm::mat4 View = glm::lookAt(
		glm::vec3(0, 0, 100), // Camera is at (4,3,-3), in World Space
		glm::vec3(0, 0, 0), // and looks at the origin
		glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
	);
	// Model matrix : an identity matrix (model will be at the origin)
	glm::mat4 Model = glm::mat4(1.0f);
	// Our ModelViewProjection : multiplication of our 3 matrices
	glm::mat4 MVP = Projection * View * Model; // Remember, matrix multiplication is the other way around

	// Our vertices. 
	// Simple four triangle representation of a bone
	static const GLfloat g_vertex_buffer_data[] = {
		 .0f, .0f, .0f,
		 .3f, .3f, .0f,
		 .3f,  0.0f, .0f,
		 .3f,  0.0f, .0f,
		 .3f,  0.3f, .0f,
		 1.0f, .0f, .0f,
		 .0f, .0f, .0f,
		 .3f, -.3f, .0f,
		 .3f,  0.0f, .0f,
		 .3f,  0.0f, .0f,
		 .3f,  -0.3f, .0f,
		 1.0f, .0f, .0f
	};

	// Color the triangles red and white.
	static const GLfloat g_color_buffer_data[] = {
		1.0f,  0.f,  0.f,
		1.0f,  0.f,  0.f,
		1.0f,  0.f,  0.f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  0.f,  0.f,
		1.0f,  0.f,  0.f,
		1.0f,  0.f,  0.f
	};

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

	GLuint colorbuffer;
	glGenBuffers(1, &colorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);


	float worldTargetX = 0.0, worldTargetY = 0.0;

	//**************************************************************************
	//**************************************************************************
	//This is the data that defines and configures the skeleton.  You can assume
	//that it starts in a horizontal line to the right, from 0,0
	//initialize the skeleton data.
	//You'll want to set the values in the theta vector using your IK algorithm
	const float lengths[] = { 13.5, 11, 3.5, 4 };
	int numBones = 4;
	//The theta vector holds the joint angles for the skeleton
	//I suggest testing that you have the rotations set up properly with some hard coded test values first.
	//float theta[] = { 0.0, 20.0, 0.0, 50.0, 0.0 };
	float theta[] = { 0.2,  0.6, 0.2, 0.3 };
	float posx[] = { 0, 0, 0, 0, 0 };
	float posy[] = { 0, 0, 0, 0, 0 };
	do {

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		// HERE
		


		/*
		theta[0] = 1;
		theta[1] = 2;
		theta[2] = 3;
		theta[3] = 4;
		*/

		//Detect when the user clicks to specify a target location (just taking button down)
		int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
		if (state == GLFW_PRESS)
		{
			double xpos, ypos;
			glfwGetCursorPos(window, &xpos, &ypos);
			//fprintf(stderr, "x %lf, y %lf\n", xpos, ypos);
			targetX = xpos;
			targetY = ypos;
			targetSet = true;

			//This should be the location you actually want to use in your IK algorithm
			worldTargetX = (targetX / 1024 - 0.5) * 120;
			worldTargetY = (-targetY / 768 + 0.5) * 120;
		}


		// Model matrix : an identity matrix (model will be at the origin)
		glm::mat4 scale, rot;
		glm::mat4 trans = glm::translate(glm::mat4(), glm::vec3(0.0f));
		glm::vec3 rotAxis = glm::vec3(0.0f, 0.0f, 1.0f);

		// Model matrix : an identity matrix (model will be at the origin)
		Model = glm::mat4(1.0f);
		MVP = Projection * View * Model; // Remember, matrix multiplication is the other way around


		//draw the target
		if (targetSet)
		{
			scale = glm::scale(glm::mat4(), glm::vec3(1.f, 10 / 6.0f, 1.0f));
			trans = glm::translate(glm::mat4(), glm::vec3(worldTargetX - .5, worldTargetY, 0.0f));//The .5 is to center the marker on the clicked location

			Model = trans * scale;
			MVP = Projection * View * Model;


			// Use our shader
			glUseProgram(programID);

			// Send our transformation to the currently bound shader, 
			// in the "MVP" uniform
			glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

			// 1rst attribute buffer : vertices
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
			glVertexAttribPointer(
				0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
				3,                  // size
				GL_FLOAT,           // type
				GL_FALSE,           // normalized?
				0,                  // stride
				(void*)0            // array buffer offset
			);

			// 2nd attribute buffer : colors
			glEnableVertexAttribArray(1);
			glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
			glVertexAttribPointer(
				1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
				3,                                // size
				GL_FLOAT,                         // type
				GL_FALSE,                         // normalized?
				0,                                // stride
				(void*)0                          // array buffer offset
			);

			// Draw the triangle !
			glDrawArrays(GL_TRIANGLES, 0, 4 * 3); // 12*3 indices starting at 0 -> 12 triangles

			glDisableVertexAttribArray(0);
			glDisableVertexAttribArray(1);


		}

		vec2 targetPosition = glm::vec2(worldTargetX, worldTargetY);

		

		

		Model = glm::mat4(1.0f);
		trans = glm::translate(glm::mat4(), glm::vec3(0.0f));
		scale = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f));

		

		if (counter > limit) {
			if (sqrt(pow(posx[4] - targetPosition[0], 2) + pow(posy[4] - targetPosition[1], 2)) > 0.2) {
				vec4 newTheta = JacobianIK(targetPosition, posx, posy, posx[4], posy[4], theta);
				theta[0] = newTheta[0];
				theta[1] = newTheta[1];
				theta[2] = newTheta[2];
				theta[3] = newTheta[3];
			}
		}
		float currentRot = 0.0;
		for (int i = 0; i < numBones; i++)
		{
			counter++;
			// Calculate the position of each of the joints
			currentRot += theta[i];
			posx[i + 1] = posx[i] + lengths[i] * cos(currentRot);
			posy[i + 1] = posy[i] + lengths[i] * sin(currentRot);
			
			
			//std::cout << (posx[i]) << std::endl;
			//std::cout << (posy[i]) << std::endl;
			
			
			
			

			if (i > 0)
			{
				scale = glm::scale(glm::mat4(), glm::vec3(1 / lengths[i - 1], 1.0 / 2.f, 1.0f));
				Model = Model * scale;
			}
			if (i > 0)
			{
				trans = glm::translate(glm::mat4(), glm::vec3(lengths[i - 1], 0.f, 0.0f));
			}
			//Note that this rotation is in radians
			rot = glm::rotate(theta[i], rotAxis);

			scale = glm::scale(glm::mat4(), glm::vec3(lengths[i], 2.0f, 1.0f));

			//*********************************************************************************************
			//NOTE: I've removed the rotation from the model matrix.  You need to figure out where to add it
			//Done 
			Model = Model * trans * rot * scale;
			//Model = Model * trans * rot * scale;
			MVP = Projection * View * Model;


			// Use our shader
			glUseProgram(programID);

			// Send our transformation to the currently bound shader, 
			// in the "MVP" uniform
			glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

			// 1rst attribute buffer : vertices
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
			glVertexAttribPointer(
				0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
				3,                  // size
				GL_FLOAT,           // type
				GL_FALSE,           // normalized?
				0,                  // stride
				(void*)0            // array buffer offset
			);

			// 2nd attribute buffer : colors
			glEnableVertexAttribArray(1);
			glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
			glVertexAttribPointer(
				1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
				3,                                // size
				GL_FLOAT,                         // type
				GL_FALSE,                         // normalized?
				0,                                // stride
				(void*)0                          // array buffer offset
			);

			// Draw the triangle !
			glDrawArrays(GL_TRIANGLES, 0, 4 * 3); // 12*3 indices starting at 0 -> 12 triangles

			glDisableVertexAttribArray(0);
			glDisableVertexAttribArray(1);



		}
		currentRot = 0.0;
		

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

	} // Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0);

	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &colorbuffer);
	glDeleteProgram(programID);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}

