#include "viewer.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
using namespace std;

namespace COL781 {
	namespace Viewer {

		namespace GL = COL781::OpenGL;

		void Camera::initialize(float aspect) {
			firstMouse = true;
			yaw   = -90.0f;	
			pitch =  0.0f;
			lastX =  800.0f / 2.0;
			lastY =  600.0 / 2.0;
			fov   =  60.0f;

			this->aspect = aspect;

			position = glm::vec3(0.0f, 0.0f,  1.5f);
			lookAt = glm::vec3(0.0f, 0.0f, 0.0f);
			up = glm::vec3(0.0f, 1.0f,  0.0f);

			updateViewMatrix();
		}

		glm::mat4 Camera::getViewMatrix() {
			return viewMatrix;
		}

		void Camera::updateViewMatrix() {
			viewMatrix = glm::lookAt(position, lookAt, up);
		}

		glm::mat4 Camera::getProjectionMatrix() {
			return glm::perspective(glm::radians(fov), aspect, 0.1f, 100.0f);
		}
			glm::vec3 getRightVector();

		glm::vec3 Camera:: getViewDir() {
			return -glm::transpose(viewMatrix)[2];
		}

		glm::vec3 Camera::getRightVector() {
			return glm::transpose(viewMatrix)[0];
		}

		void Camera::setCameraView(glm::vec3 position_vector, glm::vec3 lookat_vector, glm::vec3 up_vector) {
			position = std::move(position_vector);
			lookAt = std::move(lookat_vector);
			up = std::move(up_vector);

			viewMatrix = glm::lookAt(position, lookAt, up);
		}

		bool Viewer::initialize(const std::string &title, int width, int height) {
			if (!r.initialize(title.c_str(), width, height))
				return false;
			program = r.createShaderProgram(
				r.vsPhongShading(),
				r.fsPhongShading()
			);
			r.useShaderProgram(program);
			object = r.createObject();
			r.enableDepthTest();
			camera.initialize((float)width/(float)height);
			return true;
		}

		void Viewer::setVertices(int n, const glm::vec3* vertices) {
			r.setVertexAttribs(object, 0, n, vertices);
		}

		void Viewer::setNormals(int n, const glm::vec3* normals) {
			r.setVertexAttribs(object, 1, n, normals);
		}

		void Viewer::setTriangles(int n, const glm::ivec3* triangles) {
			r.setTriangleIndices(object, n, triangles);
		}

		void Viewer::view() {
			// The transformation matrix.
			glm::mat4 model = glm::mat4(1.0f);
			glm::mat4 view;    
			glm::mat4 projection = camera.getProjectionMatrix();

			float deltaAngleX = 2.0 * 3.14 / 800.0;
			float deltaAngleY = 3.14 / 600.0;

			int lastxPos, lastyPos, xPos, yPos;

			SDL_GetMouseState(&lastxPos, &lastyPos);
			
			//MyCode
			
			float g=10.0;
				float mass=10.0f;
				float kst = 5.0;
				float ksh = 3.0;
				float fle = 2.0;
				
				float l1= 1.0, l2= 1.414;
				vector<float> posr{-0.5,-0.5,0.0,0.5,-0.5,0.0,-0.5,0.5,0.0,0.5,0.5,0.0};
				vector<float> posc{-0.5,-0.5,0.0,0.5,-0.5,0.0,-0.5,0.5,0.0,0.5,0.5,0.0};
				vector<float> vel{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
				vector<float> force(12,0.0);
				vector<float> acc(12,0.0);
				
				
				
			/*	glm::vec presx(-0.5,0.5,-0.5,0.5);
				glm::mat2 presy(-0.5,-0.5,0.5,0.5);
				glm::mat2 pcurx(-0.5,0.5,-0.5,0.5);
				glm::mat2 pcury(-0.5,-0.5,0.5,0.5);*/
				
				
				glm::mat2 fx(0.0f);
				glm::mat2 fy(0.0f);
				glm::mat2 ax(0.0f);
				glm::mat2 ay(0.0f);
				glm::mat2 vx(0.0f);
				glm::mat2 vy(0.0f);
				
				float delt=0.001f;

			while (!r.shouldQuit()) {
				r.clear(glm::vec4(1.0, 1.0, 1.0, 1.0));

				camera.updateViewMatrix();
				
				//Mycode
				float x0=posc[0]-posc[3];
				float y0=posc[1]-posc[4];
				float t0=sqrt((x0*x0)+(y0*y0));
				x0/=t0, y0/=t0;
				float f0=sqrt(pow((posc[0]-posc[3]),2)+pow((posc[1]-posc[4]),2))-l1;
				force[0]-=kst*f0*x0;
				force[1]-=kst*f0*y0;
				force[3]+=kst*f0*x0;
				force[4]+=kst*f0*y0;
				
				float x1=posc[0]-posc[6];
				float y1=posc[1]-posc[7];
				float t1=sqrt((x1*x1)+(y1*y1));
				x1/=t1, y1/=t1;
				float f1=sqrt(pow((posc[0]-posc[6]),2)+pow((posc[1]-posc[7]),2))-l1;
				force[0]-=kst*f1*x1;
				force[1]-=kst*f1*y1;
				force[6]+=kst*f1*x1;
				force[7]+=kst*f1*y1;
				
				float x2=posc[0]-posc[9];
				float y2=posc[1]-posc[10];
				float t2=sqrt((x2*x2)+(y2*y2));
				x2/=t2, y2/=t2;
				float f2=sqrt(pow((posc[0]-posc[9]),2)+pow((posc[1]-posc[10]),2))-l2;
				force[0]-=ksh*f2*x2;
				force[1]-=ksh*f2*y2;
				force[9]+=ksh*f2*x2;
				force[10]+=ksh*f2*y2;
				
				float x3=posc[3]-posc[9];
				float y3=posc[4]-posc[10];
				float t3=sqrt((x3*x3)+(y3*y3));
				x3/=t3, y3/=t3;
				float f3=sqrt(pow((posc[3]-posc[9]),2)+pow((posc[4]-posc[10]),2))-l1;
				force[3]-=kst*f3*x3;
				force[4]-=kst*f3*y3;
				force[9]+=kst*f3*x3;
				force[10]+=kst*f3*y3;
				
				float x4=posc[3]-posc[6];
				float y4=posc[4]-posc[7];
				float t4=sqrt((x4*x4)+(y4*y4));
				x4/=t4, y4/=t4;
				float f4=sqrt(pow((posc[3]-posc[6]),2)+pow((posc[4]-posc[7]),2))-l1;
				force[3]-=ksh*f4*x4;
				force[4]-=ksh*f4*y4;
				force[6]-=ksh*f4*x4;
				force[7]-=ksh*f4*y4;
				
				float x5=posc[6]-posc[9];
				float y5=posc[7]-posc[10];
				float t5=sqrt((x5*x5)+(y5*y5));
				x5/=t5, y5/=t5;
				float f5=sqrt(pow((posc[6]-posc[9]),2)+pow((posc[7]-posc[10]),2))-l1;
				force[6]-=kst*f5*x5;
				force[7]-=kst*f5*y5;
				force[9]-=kst*f5*x5;
				force[10]-=kst*f5*y5;
				
				force[1]+=mass*g;
				force[4]+=mass*g;
				force[7]+=mass*g;
				force[10]+=mass*g;
				
				for(int i=0;i<12;i++)
				{	
					if(i%4 != 0 && i%4 !=1){ 
					acc[i]=force[i]/mass;
					vel[i]+=acc[i]*delt;
					posc[i]+=vel[i]*delt;
					}
				}
				glm::vec3 vertices[] ={
				glm::vec3(posc[0], posc[1], 0.0),
				glm::vec3(posc[3], posc[4], 0.0),
				glm::vec3(posc[6], posc[7], 0.0),
				glm::vec3( 0.5,  0.5, 0.0)
			    	};
			    
			    glm::vec3 vertices1[] ={
				glm::vec3(posc[3], posc[4], 0.0),
				glm::vec3(posc[9], posc[10], 0.0),
				glm::vec3(posc[6],  posc[7], 0.0),
				glm::vec3( 0.5,  0.5, 0.0)
			    	};
				setVertices(4, vertices);
				setVertices(4, vertices1);
				
				

				/*for(int i=0;i<2;i++)
				{
					for(int j=0;j<2;j++)
					{
						int a=pcurx[i][j], b=pcury[i][j];
						if(i+1<=1)
						{
							f[i][j]-=kst* sqrt(pow((pcurx[i+1][j]-pcurx[i][j]),2)+pow((pcurx[i+1][j]-pcurx[i+1][j]),2))-l1;
							fy[i][j]-=kst*(-presy[i][j]);
							
						}
						if(j+1<=1)
						{
							fx[i][j]-=kst*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=kst*(pcury[i][j]-presy[i][j]);
						}
						if(i-1>=0)
						{
							fx[i][j]-=kst*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=kst*(pcury[i][j]-presy[i][j]);
						}
						if(j-1>=0)
						{
							fx[i][j]-=kst*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=kst*(pcury[i][j]-presy[i][j]);
						}
						if(i+1<=1 && j+1<=1)
						{
							fx[i][j]-=ksh*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=ksh*(pcury[i][j]-presy[i][j]);
						}
						if(i+1<=1 && j-1>=0)
						{
							fx[i][j]-=ksh*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=ksh*(pcury[i][j]-presy[i][j]);
						}
						if(i-1>=0 && j+1<=1)
						{
							fx[i][j]-=ksh*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=ksh*(pcury[i][j]-presy[i][j]);
						}
						if(i-1>=0 && j-1>=0)
						{
							fx[i][j]-=ksh*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=ksh*(pcury[i][j]-presy[i][j]);
						}
						if(i+2<=1)
						{
							fx[i][j]-=fle*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=fle*(pcury[i][j]-presy[i][j]);
						}
						if(j+2<=1)
						{
							fx[i][j]-=fle*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=fle*(pcury[i][j]-presy[i][j]);
						}
						if(i-2>=0)
						{
							fx[i][j]-=fle*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=fle*(pcury[i][j]-presy[i][j]);
						}
						if(j-2>=0)
						{
							fx[i][j]-=fle*(pcurx[i][j]-presx[i][j]);
							fy[i][j]-=fle*(pcury[i][j]-presy[i][j]);
						}
						fy[i][j]+=mass*g;
						ax[i][j]=fx[i][j]/mass;
						ay[i][j]=fy[i][j]/mass;
						vx[i][j]+=ax[i][j]*delt;
						vy[i][j]+=ay[i][j]*delt;
						pcurx[i][j]+=vx[i][j]*delt;
						pcury[i][j]+=vy[i][j]*delt;
					}
				}
				for(int i=0;i<=0;i++)
        			{
        				for(int j=0;j<=0;j++)
					{
						glm::vec3 vertices[] ={
						glm::vec3(pcurx[i][j], pcury[i][j], 0.0),
						glm::vec3(pcurx[i+1][j+1], pcury[i+1][j+1], 0.0),
						glm::vec3(pcurx[i][j+1],  pcury[i][j+1], 0.0),
						glm::vec3( 0.5,  0.5, 0.0)
					    	};
					    
					    glm::vec3 vertices1[] ={
						glm::vec3(pcurx[i][j], pcury[i][j], 0.0),
						glm::vec3(pcurx[i+1][j], pcury[i+1][j], 0.0),
						glm::vec3(pcurx[i+1][j+1],  pcury[i+1][j+1], 0.0),
						glm::vec3( 0.5,  0.5, 0.0)
					    	};
						setVertices(4, vertices);
						setVertices(4, vertices1);
					}
				}*/

				Uint32 buttonState = SDL_GetMouseState(&xPos, &yPos);
				if( buttonState & SDL_BUTTON(SDL_BUTTON_LEFT) ) {
					glm::vec4 pivot = glm::vec4(camera.lookAt.x, camera.lookAt.y, camera.lookAt.z, 1.0f);
					glm::vec4 position = glm::vec4(camera.position.x, camera.position.y, camera.position.z, 1.0f);

					float xAngle = (float)(lastxPos - xPos) * deltaAngleX;
					float yAngle = (float)(lastyPos - yPos) * deltaAngleY;

					float cosAngle = dot(camera.getViewDir(), camera.up);

					if(cosAngle * signbit(deltaAngleY) > 0.99f)
						deltaAngleY = 0.0f;

					glm::mat4 rotationMatX(1.0f);
					rotationMatX = glm::rotate(rotationMatX, xAngle, camera.up);
					position = (rotationMatX * (position - pivot)) + pivot;

					glm::mat4 rotationMatY(1.0f);
					rotationMatY = glm::rotate(rotationMatY, yAngle, camera.getRightVector());
					glm::vec3 finalPosition = (rotationMatY * (position - pivot)) + pivot;
					camera.position = finalPosition;
					camera.updateViewMatrix();
				}

				lastxPos = xPos;
				lastyPos = yPos;

				view = camera.getViewMatrix();

				r.setUniform(program, "modelView", view*model);
				r.setUniform(program, "projection", projection);
				r.setUniform(program, "lightPos", camera.position);
				r.setUniform(program, "viewPos", camera.position);
				r.setUniform(program, "lightColor", glm::vec3(1.0f, 1.0f, 1.0f));

				r.setupFilledFaces();
				r.setUniform(program, "objectColor", glm::vec3(1.0f, 1.0f, 1.0f));
				r.drawObject(object);

				r.setupWireFrame();
				r.setUniform(program, "objectColor", glm::vec3(0.0f, 0.0f, 0.0f));
				r.drawObject(object);
				r.show();
			}
		}

	}
}

		// void Rasterizer::show(Camera* cam) {
		// 	SDL_GL_SwapWindow(window);
		// 	SDL_Event e;
		// 	while (SDL_PollEvent(&e) != 0) {
		// 		if(e.type == SDL_QUIT) {
		// 			quit = true;
		// 		}
		// 		else if(e.type == SDL_KEYDOWN) {
		// 			//TODO clean
		// 			// std::cout<<e.key.keysym.sym<<" key pressed"<<std::endl;
		// 			int key = e.key.keysym.sym;
		// 			if(key == 119) {
		// 				cam->cameraPos += cam->cameraSpeed * cam->cameraFront;
		// 			}
		// 			else if(key == 115) {
		// 				cam->cameraPos -= cam->cameraSpeed * cam->cameraFront;
		// 			}
		// 			else if(key == 97) {
		// 				cam->cameraPos -= cam->cameraSpeed * glm::normalize(glm::cross(cam->cameraFront, cam->cameraUp));
		// 			}
		// 			else if(key == 100) {
		// 				cam->cameraPos += cam->cameraSpeed * glm::normalize(glm::cross(cam->cameraFront, cam->cameraUp));
		// 			}
		// 		}
		// 		else if(e.type == SDL_MOUSEMOTION) {
		// 			int x, y;
		// 			float xpos, ypos;
		// 			SDL_GetGlobalMouseState(&x, &y);
		// 			xpos = (float)x;
		// 			ypos = (float)y;	

		// 			if(cam->firstMouse) {
		// 				cam->lastX = xpos;
		// 				cam->lastY = ypos;
		// 				cam->firstMouse = false;
		// 			}

		// 			float xoffset = xpos - cam->lastX;
		// 			float yoffset = cam->lastY - ypos; // reversed since y-coordinates go from bottom to top
		// 			cam->lastX = xpos;
		// 			cam->lastY = ypos;

		// 			    float sensitivity = 0.1f; // change this value to your liking
		// 				xoffset *= sensitivity;
		// 				yoffset *= sensitivity;

		// 				cam->yaw += xoffset;
		// 				cam->pitch += yoffset;

		// 				// make sure that when pitch is out of bounds, screen doesn't get flipped
		// 				if (cam->pitch > 89.0f)
		// 					cam->pitch = 89.0f;
		// 				if (cam->pitch < -89.0f)
		// 					cam->pitch = -89.0f;

		// 				glm::vec3 front;
		// 				front.x = cos(glm::radians(cam->yaw)) * cos(glm::radians(cam->pitch));
		// 				front.y = sin(glm::radians(cam->pitch));
		// 				front.z = sin(glm::radians(cam->yaw)) * cos(glm::radians(cam->pitch));
		// 				cam->cameraFront = glm::normalize(front);
		// 		}
		// 		else if(e.type == SDL_MOUSEWHEEL) {
		// 			cam->fov -= (float)e.wheel.y;
		// 			if (cam->fov < 1.0f)
		// 				cam->fov = 1.0f;
		// 			if (cam->fov > 45.0f)
		// 				cam->fov = 45.0f;
		// 		}
		// 	}
		// 	glCheckError();
		// }
