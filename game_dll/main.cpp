//#####################################################################
// Copyright 2002-2007, Ronald Fedkiw, Eran Guendelman, Tamar Shinar, Joseph Teran.
// This file is part of PhysBAM whose distribution is governed by the license contained in the accompanying file PHYSBAM_COPYRIGHT.txt.
//#####################################################################
#include <PhysBAM_Tools/Parallel_Computation/MPI_WORLD.h>
#include <PhysBAM_Dynamics/SOLIDS_FLUIDS_INTERACTIVE_DRIVER_UNIFORM.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_BASIC_CALLBACKS.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/ANIMATED_VISUALIZATION.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/BASIC_VISUALIZATION.h>
#include "STRO_NEW.h"
#include "opengl_interactive.h"
#include <pthread.h>
#include <stdio.h>
#ifndef WIN32
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#endif
#include <sys/types.h>
//
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_TEXTURED_RECT_3D.h>
#include <PhysBAM_Tools/Parallel_Computation/PTHREAD.h>
#include <C:\Army2015\PhysDrive-PC\Public_Library\PhysBAM_Solids\PhysBAM_Rigids\Collisions\RIGID_BODY_INTERSECTIONS.h>
#include <sstream>

using namespace PhysBAM;

typedef float T;
typedef float RW;
typedef VECTOR<T,3> TV;
STRO_NEW<T>* rigids_example=0;
SOLIDS_FLUIDS_INTERACTIVE_DRIVER_UNIFORM<GRID<TV> >* driver;
INTERACTIVE_VISUALIZATION<T, RW> *visualizer=0;
int need_fog = 0;

void *Execute_Driver(void *driver) {
    typedef float T;
    typedef VECTOR<T,3> TV;
    //setpriority(PRIO_PROCESS, getpid(), 3);
//    rigids_example->destroy_jeep();
    int explode_frame = -1;
    for (unsigned int i = 1; true; ++i) {
    	if (rigids_example->condition == 1) {
    		visualizer->opengl_world.has_success = rigids_example->condition;
    		break;
    	}
    	((SOLIDS_FLUIDS_INTERACTIVE_DRIVER_UNIFORM<GRID<TV> >*)driver)->Simulate_To_Frame(i);
    	if (rigids_example->condition == 3 && explode_frame < 0)
    		explode_frame = i;
    	if ((rigids_example->jeep->X()(2) < -50) || (rigids_example->condition == 3 && i - explode_frame > 200 && rigids_example->is_shaking == 0)) {
    		visualizer->opengl_world.has_success = 2;
    		break;
    	}
	}
    int *retval = new int;
    *retval = 0;
    pthread_exit((void *) retval);

    return 0;
}

void *Visualize(void *visualizer) {
    typedef float T;
    typedef float RW;
    //setpriority(PRIO_PROCESS, getpid(), 2);
    Initialize_Rigids_Particles();
    Initialize_Read_Write_Structures();
    //LOG::Initialize_Logging(false,false,1<<30,true,1);
    //LOG::Initialize_Logging(true,true,0,true); 
    PROCESS_UTILITIES::Set_Floating_Point_Exception_Handling(true);
    PROCESS_UTILITIES::Set_Backtrace(true);
    ((INTERACTIVE_VISUALIZATION<T,RW>*)visualizer)->Run();
//    pthread_exit(0);
    return 0;
}

VECTOR<float,3> getCameraDirection() {
    VECTOR<float,3> camera,target,up;
    visualizer->opengl_world.Get_Look_At(camera,target,up);
	return target - camera;
}


void initVisualizer() {
    visualizer->Parse_Args(visualizer->argc,visualizer->argv);
    // visualizer->PreInitialize_OpenGL_World();
    visualizer->Initialize_Components_And_Key_Bindings();

	driver->rigid_component=dynamic_cast<OPENGL_COMPONENT_RIGID_BODY_COLLECTION_3D<T>*>(visualizer->Find_Component("Rigid Bodies"));
	driver->rigid_component->rigid_body_collection_simulated=&(rigids_example->solid_body_collection.rigid_body_collection);
	driver->rigid_component->rigid_geometry_collection_simulation=&(rigids_example->solid_body_collection.rigid_body_collection.rigid_geometry_collection);
	driver->rigid_component->rigid_geometry_collection=new RIGID_GEOMETRY_COLLECTION<TV>(*new RIGID_GEOMETRY_PARTICLES<TV>(),0);
	driver->rigid_component->rigid_geometry_collection_rendering = new RIGID_GEOMETRY_COLLECTION<TV>(*new RIGID_GEOMETRY_PARTICLES<TV>(),0);

	// visualizer->opengl_world.Bind_Key('j', new OPENGL_CALLBACK_TOGGLE_JEEP(&(driver->rigid_component->jeep_display_mode), driver->rigid_component->rigid_geometry_collection));

}

void setCameraShakeMagnitude(float magnitude) {
	visualizer->Set_Camera_Shake_Magnitude(magnitude);
}


int cur_frame = 1;
int explode_frame = -1;

void Init()
{
	srand((unsigned)time(NULL));
    typedef float T;
    typedef float RW;
    typedef VECTOR<T,3> TV;
    STREAM_TYPE stream_type((RW()));
    // pthread_t driver_thread;
	rigids_example=new STRO_NEW<T>(stream_type);
    rigids_example->want_mpi_world=true;
    rigids_example->write_output_files=false;
    int level = 0;
	int argc = 0;
	char *argv[5];
	if (argc > 2) {
		sscanf(argv[2], "%d", &need_fog);
	    argv[--argc] = 0;
	}
    if (argc > 1) {
	    sscanf(argv[1], "%d", &level);
	    argv[--argc] = 0;
	}
	if (level == 1)
		level = 0;
	//level = 3;
    rigids_example->level = level;
	char str[] = "haha\0"; argv[0] = str;
    rigids_example->Parse(argc,argv);

#ifdef USE_MPI
    if (rigids_example->mpi_world->initialized) {
        rigids_example->mpi_rigids=new MPI_RIGIDS<TV>();
    }
        //rigids_example->Adjust_Output_Directory_For_MPI(rigids_example->mpi_rigids);}
#endif

    visualizer=new INTERACTIVE_VISUALIZATION<T,RW>();

    visualizer->argc=argc;
    visualizer->argv=argv;
    //visualizer->is_interactive = true; // TODO make an option later
    // visualizer->opengl_world.Set_Idle_Callback(new OPENGL_CALLBACK_REDISPLAY(visualizer->opengl_world),0);
    //visualizer->data_lock=new pthread_mutex_t;
    //pthread_mutex_init(visualizer->data_lock,0);
    //visualizer->opengl_world.animated_visualization = visualizer;

    driver = new SOLIDS_FLUIDS_INTERACTIVE_DRIVER_UNIFORM<GRID<TV> >(*rigids_example);
    rigids_example->driver = driver;

    driver->visualizer=visualizer;
    driver->Initialize();

	//visualizer->speed = &(rigids_example->curr_speed);
	//visualizer->reversing = &(rigids_example->reverse);
	//visualizer->Goto_Start_Frame();
 //   visualizer->PostInitialize_OpenGL_World();
 //   visualizer->Initialize_Lights();
	rigids_example->solids_evolution->rigid_body_collisions->collision_manager = rigids_example->collision_manager;

    //pthread_create(&driver_thread, NULL, Execute_Driver, (void*) driver);
    // Visualize((void*) visualizer);

    // Cancel the other thread and wait for it to finish.
    //pthread_cancel(driver_thread);
    //pthread_join(driver_thread, NULL);

	cur_frame = 1;
	explode_frame = -1;
}

string Get_Body_State(RIGID_BODY<TV> *rd)
{
	stringstream ss;
	ss << "pos:[" << rd->X().x << "," << rd->X().y << "," << rd->X().z << "],";
	TV ori = rd->Rotation().Euler_Angles();
	ss << "ori:[" << ori.x << "," << ori.y << "," << ori.z << "]";
	return ss.str();
}


void Put_Jeep_State(ostream &output)
{
	output << "{" << Get_Body_State(rigids_example->jeep) << "}" << endl;
}

char *info_str = NULL;

void Put_Jeep_State(stringstream &output)
{
	output << "{" << Get_Body_State(rigids_example->jeep) << "}";
}

char* Get_Info_str()
{
	stringstream ss;
	Put_Jeep_State(ss);
	if (info_str){ try{delete info_str;} catch (...){}}
	string s = ss.str();
	info_str = new char[s.length()+1];
	strcpy(info_str, s.c_str());
	return info_str;
}

void Move_Foward()
{
	bool *forward = &rigids_example->forward, *back = &rigids_example->back;
	int *curr_accel = &rigids_example->curr_accel;
	*forward = true;
	*back = false;
	if (*curr_accel >= 20){
		*curr_accel = 20;
	}
	else *curr_accel += 1;
}

char * Update_Frame()
{
	if (rigids_example->condition == 1) {
		visualizer->opengl_world.has_success = rigids_example->condition;
	}
	Move_Foward();
	((SOLIDS_FLUIDS_INTERACTIVE_DRIVER_UNIFORM<GRID<TV> >*)driver)->Simulate_To_Frame(cur_frame++);
	if (rigids_example->condition == 3 && explode_frame < 0)
		explode_frame = cur_frame;
	if ((rigids_example->jeep->X()(2) < -50) || (rigids_example->condition == 3 && cur_frame - explode_frame > 200 && rigids_example->is_shaking == 0)) {
		visualizer->opengl_world.has_success = 2;
	}
	return Get_Info_str();
}

int main()
{
	Init();
	while (true)
	{
		cout << Update_Frame() << endl;
	}
}

//extern "C"
//{
//	void API_Init(){ Init(); }
//	char *API_Update_Frame(){ Update_Frame(); }
//}