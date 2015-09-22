#include "application.h"
#include "Timer.h"
#include <iostream>
#include <gl/glut.h>
#include <stdio.h>

// GLFW
#include <GLFW/glfw3.h>
#ifdef _MSC_VER
#undef APIENTRY
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#include <GLFW/glfw3native.h>
#endif

#pragma region IMGUI

#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))

// Imgui
#include "../Imgui/imgui.h"

static GLFWwindow* windowG;
static bool mousePressed[2] = { false, false };

/*
	--ImGui--
*/

/*
	This is the main function that you have to implment and provide to ImGui
	(via settign up "RenderDrawListsFn" in the ImGuiIO structure)
	If test of lines are blurrry when intergrating ImGui in your engine:
		- in your render function, try translating your projection matrix by (0.5f, 0.5f) or (0.375f, 0.375f)
*/
static void ImImpl_RenderDrawLists(ImDrawList** const cmd_lists, int cmd_lists_count)
{
	if (cmd_lists_count == 0)
		return;

	// We are using the OpenGL fixed pipeline to make the example code simpler to read!
	// A probable faster way to render would be to collate all vertices from all cmd_lists into a single vertex buffer.
	// Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, vertex/texcoord/color pointers.
	glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_SCISSOR_TEST);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnable(GL_TEXTURE_2D);

	// Setup orthographic projection matrix
	const float width = ImGui::GetIO().DisplaySize.x;
	const float height = ImGui::GetIO().DisplaySize.y;
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0f, width, height, 0.0f, -1.0f, +1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// Render command lists
	for (int n = 0; n < cmd_lists_count; n++)
	{
		const ImDrawList* cmd_list = cmd_lists[n];
		const unsigned char* vtx_buffer = (const unsigned char*)&cmd_list->vtx_buffer.front();
		glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)(vtx_buffer + OFFSETOF(ImDrawVert, pos)));
		glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)(vtx_buffer + OFFSETOF(ImDrawVert, uv)));
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert), (void*)(vtx_buffer + OFFSETOF(ImDrawVert, col)));

		int vtx_offset = 0;
		for (size_t cmd_i = 0; cmd_i < cmd_list->commands.size(); cmd_i++)
		{
			const ImDrawCmd* pcmd = &cmd_list->commands[cmd_i];
			glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->texture_id);
			glScissor((int)pcmd->clip_rect.x, (int)(height - pcmd->clip_rect.w), (int)(pcmd->clip_rect.z - pcmd->clip_rect.x), (int)(pcmd->clip_rect.w - pcmd->clip_rect.y));
			glDrawArrays(GL_TRIANGLES, vtx_offset, pcmd->vtx_count);
			vtx_offset += pcmd->vtx_count;
		}
	}

	// Restore modified state
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}

// NB: ImGui already provide OS clipboard support for Windows so this isn't needed if you are using Windows only.
static const char* ImImpl_GetClipboardTextFn()
{
	return glfwGetClipboardString(windowG);
}

static void ImImpl_SetClipboardTextFn(const char* text)
{
	glfwSetClipboardString(windowG, text);
}

// GLFW callbacks
static void glfw_error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS && button >= 0 && button < 2)
		mousePressed[button] = true;
}

static void glfw_scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MouseWheel += (float)yoffset; // Use fractional mouse wheel, 1.0 unit 5 lines.
}

static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	ImGuiIO& io = ImGui::GetIO();
	if (action == GLFW_PRESS)
		io.KeysDown[key] = true;
	if (action == GLFW_RELEASE)
		io.KeysDown[key] = false;
	io.KeyCtrl = (mods & GLFW_MOD_CONTROL) != 0;
	io.KeyShift = (mods & GLFW_MOD_SHIFT) != 0;
}

static void glfw_char_callback(GLFWwindow* window, unsigned int c)
{
	if (c > 0 && c < 0x10000)
		ImGui::GetIO().AddInputCharacter((unsigned short)c);
}

void InitGL()
{
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		exit(1);

	windowG = glfwCreateWindow(1280, 720, "Physics_Engine by Darren Sweeney", NULL, NULL);
	glfwMakeContextCurrent(windowG);
	glfwSetKeyCallback(windowG, glfw_key_callback);
	glfwSetMouseButtonCallback(windowG, glfw_mouse_button_callback);
	glfwSetScrollCallback(windowG, glfw_scroll_callback);
	glfwSetCharCallback(windowG, glfw_char_callback);
}

void LoadFontsTexture()
{
	ImGuiIO& io = ImGui::GetIO();
	//ImFont* my_font1 = io.Fonts->AddFontDefault();
	//ImFont* my_font2 = io.Fonts->AddFontFromFileTTF("extra_fonts/Karla-Regular.ttf", 15.0f);
	//ImFont* my_font3 = io.Fonts->AddFontFromFileTTF("extra_fonts/ProggyClean.ttf", 13.0f); my_font3->DisplayOffset.y += 1;
	//ImFont* my_font4 = io.Fonts->AddFontFromFileTTF("extra_fonts/ProggyTiny.ttf", 10.0f); my_font4->DisplayOffset.y += 1;
	//ImFont* my_font5 = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, io.Fonts->GetGlyphRangesJapanese());

	unsigned char* pixels;
	int width, height;
	io.Fonts->GetTexDataAsAlpha8(&pixels, &width, &height);

	GLuint tex_id;
	glGenTextures(1, &tex_id);
	glBindTexture(GL_TEXTURE_2D, tex_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, width, height, 0, GL_ALPHA, GL_UNSIGNED_BYTE, pixels);

	// Store our identifier
	io.Fonts->TexID = (void *)(intptr_t)tex_id;
}

void InitImGui()
{
	ImGuiIO& io = ImGui::GetIO();
	io.DeltaTime = 1.0f / 60.0f;                                    // Time elapsed since last frame, in seconds (in this sample app we'll override this every frame because our time step is variable)
	io.KeyMap[ImGuiKey_Tab] = GLFW_KEY_TAB;                       // Keyboard mapping. ImGui will use those indices to peek into the io.KeyDown[] array.
	io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
	io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
	io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
	io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
	io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
	io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
	io.KeyMap[ImGuiKey_Delete] = GLFW_KEY_DELETE;
	io.KeyMap[ImGuiKey_Backspace] = GLFW_KEY_BACKSPACE;
	io.KeyMap[ImGuiKey_Enter] = GLFW_KEY_ENTER;
	io.KeyMap[ImGuiKey_Escape] = GLFW_KEY_ESCAPE;
	io.KeyMap[ImGuiKey_A] = GLFW_KEY_A;
	io.KeyMap[ImGuiKey_C] = GLFW_KEY_C;
	io.KeyMap[ImGuiKey_V] = GLFW_KEY_V;
	io.KeyMap[ImGuiKey_X] = GLFW_KEY_X;
	io.KeyMap[ImGuiKey_Y] = GLFW_KEY_Y;
	io.KeyMap[ImGuiKey_Z] = GLFW_KEY_Z;

	io.RenderDrawListsFn = ImImpl_RenderDrawLists;
	io.SetClipboardTextFn = ImImpl_SetClipboardTextFn;
	io.GetClipboardTextFn = ImImpl_GetClipboardTextFn;
#ifdef _MSC_VER
	io.ImeWindowHandle = glfwGetWin32Window(windowG);
#endif

	LoadFontsTexture();
}

void UpdateImGui()
{
	ImGuiIO& io = ImGui::GetIO();

	// Setup resolution (every frame to accommodate for window resizing)
	int w, h;
	int display_w, display_h;
	glfwGetWindowSize(windowG, &w, &h);
	glfwGetFramebufferSize(windowG, &display_w, &display_h);
	io.DisplaySize = ImVec2((float)display_w, (float)display_h);                                   // Display size, in pixels. For clamping windows positions.

	// Setup time step
	static double time = 0.0f;
	const double current_time = glfwGetTime();
	io.DeltaTime = (float)(current_time - time);
	time = current_time;

	// Setup inputs
	// (we already got mouse wheel, keyboard keys & characters from glfw callbacks polled in glfwPollEvents())
	double mouse_x, mouse_y;
	glfwGetCursorPos(windowG, &mouse_x, &mouse_y);
	mouse_x *= (float)display_w / w;                                                               // Convert mouse coordinates to pixels
	mouse_y *= (float)display_h / h;
	io.MousePos = ImVec2((float)mouse_x, (float)mouse_y);                                          // Mouse position, in pixels (set to -1,-1 if no mouse / on another screen, etc.)
	io.MouseDown[0] = mousePressed[0] || glfwGetMouseButton(windowG, GLFW_MOUSE_BUTTON_LEFT) != 0;  // If a mouse press event came, always pass it as "mouse held this frame", so we don't miss click-release events that are shorter than 1 frame.
	io.MouseDown[1] = mousePressed[1] || glfwGetMouseButton(windowG, GLFW_MOUSE_BUTTON_RIGHT) != 0;

	// Start the frame
	ImGui::NewFrame();
}

/*
	--END IMGUI--
*/

#pragma endregion Code for setting up IMGUI

void Application::Init()
{
	width = 1900;
	height = 1000;

	InitGL();
	InitImGui();

	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	SetView();

	std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
}

GLFWwindow *Application::getWindow()
{
	return windowG;
}

void Application::SetView()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0f, (double)width / (double)height, 1.0f, 500.0);
	glMatrixMode(GL_MODELVIEW);
}

void Application::Update() {}

bool Application::Key(int key) 
{
	int state = glfwGetKey(windowG, key);
	if (state == GLFW_PRESS)
		return true;
	else
		return false;
}

void Application::Render() 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(0.0, 3.5, 8.0, 0.0, 3.5, 0.0, 0.0, 1.0, 0.0);
}

void Application::CleanUp()
{
	ImGui::Shutdown();
	glfwTerminate();
}

int Application::Execute()
{
	Init();

	// First demo to appear on screen!
	switchDemos[1] = true;

	bool show_physics_window = true;
	ImVec4 clear_col = ImColor(114, 144, 154);

	while (!glfwWindowShouldClose(windowG))
	{
		ImGuiIO& io = ImGui::GetIO();
		mousePressed[0] = mousePressed[1] = false;
		glfwPollEvents();
		UpdateImGui();

		if (show_physics_window)
		{
			static bool no_titlebar = false;
			static bool no_border = true;
			static bool no_resize = false;
			static bool no_move = false;
			static bool no_scrollbar = false;
			static bool no_collapse = false;
			static float bg_alpha = 0.65f;

			// Demonstrate the various window flags. Typically you would just use the default.
			ImGuiWindowFlags window_flags = 0;
			if (no_titlebar)  window_flags |= ImGuiWindowFlags_NoTitleBar;
			if (!no_border)   window_flags |= ImGuiWindowFlags_ShowBorders;
			if (no_resize)    window_flags |= ImGuiWindowFlags_NoResize;
			if (no_move)      window_flags |= ImGuiWindowFlags_NoMove;
			if (no_scrollbar) window_flags |= ImGuiWindowFlags_NoScrollbar;
			if (no_collapse)  window_flags |= ImGuiWindowFlags_NoCollapse;
			if (!ImGui::Begin("3D_Physics_Engine by Darren Sweeney", &show_physics_window, ImVec2(500, 600), bg_alpha, window_flags))
			{
				// Early out if the window is collapsed, as an optimization.
				ImGui::End();
			}

			ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.65f);

			if (ImGui::CollapsingHeader("Demos", NULL, true, true))
			{
				if (ImGui::TreeNode("Cloth"))
				{
					if (ImGui::Button("Cloth Demo"))
					{
						switchDemos[1] = true;

						switchDemos[0] = false;
						switchDemos[2] = false;
						switchDemos[3] = false;
						switchDemos[4] = false;
					}
					ImGui::Checkbox("Debug Draw", &debugDrawMode);

					ImGui::TreePop();
				}

				if (ImGui::TreeNode("Aircraft"))
				{
					if (ImGui::Button("Aircraft Demo"))
					{
						switchDemos[0] = true;

						switchDemos[1] = false;
						switchDemos[2] = false;
						switchDemos[3] = false;
						switchDemos[4] = false;
					}
					ImGui::TreePop();
				}

				if (ImGui::TreeNode("Bridge"))
				{
					if (ImGui::Button("Bridge Demo"))
					{
						switchDemos[3] = true;

						switchDemos[0] = false;
						switchDemos[2] = false;
						switchDemos[1] = false;
						switchDemos[4] = false;
					}
					ImGui::TreePop();
				}

				if (ImGui::TreeNode("Projectile"))
				{
					if (ImGui::Button("Projectile Demo"))
					{
						switchDemos[2] = true;

						switchDemos[0] = false;
						switchDemos[1] = false;
						switchDemos[3] = false;
						switchDemos[4] = false;
					}
					ImGui::TreePop();
				}

				if (ImGui::TreeNode("Collision Test"))
				{
					if (ImGui::Button("Collision Test Demo"))
					{
						switchDemos[4] = true;

						switchDemos[2] = false;
						switchDemos[0] = false;
						switchDemos[1] = false;
						switchDemos[3] = false;
					}
					ImGui::TreePop();
				}
			}

			if (ImGui::CollapsingHeader("About", NULL, true, true))
			{
				if (getDemo(0))	// Aircraft Demo.
					ImGui::TextWrapped("The right and left arrow keys control the Rudder. Keys W, S and A, D control movement of the Aileron, there is no Elevator in this simulation. The lift is genetated by the aerodynamic surface of the Aileron.");

				if (switchDemos[1])	// Cloth Demo.
					ImGui::TextWrapped("Use WASD to control the ball on screen. The cloth uses structrual, shear and bend constraints to shape the cloth under external forces. The cloth can self interpenetrate.\n\nDebug Mode: Draws normals, attached points.");

				if (getDemo(2))	// Projectile Demo.
					ImGui::TextWrapped("Use Keys 1, 2, 3, 4 to switch between different projectiles and left mouse click to shoot.");

				if (getDemo(3))	// Bridge Demo.
					ImGui::TextWrapped("Demonstration of particle constraints to make a bridge structure.");

				if (getDemo(4))	// Collision Test Demo.
					ImGui::TextWrapped("Collision Test Demo.\nPress 'R' to reset the scene.");
				
				ImGui::TextWrapped("\n\nThis is an early look into my 3D physics engine that I am working on.");
			}

			ImGui::End();
		}

		// Rendering
		glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
		glClearColor(0.9f, 0.95f, 1.0f, 1.0f);
		Render();
		ImGui::Render();

		glfwSwapBuffers(windowG);

		glColor3f(1, 0, 0);
		glPushMatrix();
		glutSolidSphere(5, 20, 20);
		glPopMatrix();

		Update();
	}

	CleanUp();

	return 0;
}

bool Application::getDemo(unsigned int index)
{
	return switchDemos[index];
}