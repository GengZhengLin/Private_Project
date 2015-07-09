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
    LOG::Initialize_Logging(true,true,0,true); 
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
    visualizer->PreInitialize_OpenGL_World();
    visualizer->Initialize_Components_And_Key_Bindings();
    // called below //   visualizer->Initialize_Lights();
	visualizer->Initialize_Drive_Callbacks(rigids_example);		//add new callbacks

	driver->rigid_component=dynamic_cast<OPENGL_COMPONENT_RIGID_BODY_COLLECTION_3D<T>*>(visualizer->Find_Component("Rigid Bodies"));
	driver->rigid_component->rigid_body_collection_simulated=&(rigids_example->solid_body_collection.rigid_body_collection);
	driver->rigid_component->rigid_geometry_collection_simulation=&(rigids_example->solid_body_collection.rigid_body_collection.rigid_geometry_collection);
	driver->rigid_component->rigid_geometry_collection=new RIGID_GEOMETRY_COLLECTION<TV>(*new RIGID_GEOMETRY_PARTICLES<TV>(),0);
	driver->rigid_component->rigid_geometry_collection_rendering = new RIGID_GEOMETRY_COLLECTION<TV>(*new RIGID_GEOMETRY_PARTICLES<TV>(),0);

	visualizer->opengl_world.Bind_Key('j', new OPENGL_CALLBACK_TOGGLE_JEEP(&(driver->rigid_component->jeep_display_mode), driver->rigid_component->rigid_geometry_collection));

}

void setCameraShakeMagnitude(float magnitude) {
	visualizer->Set_Camera_Shake_Magnitude(magnitude);
}

int main(int argc,char* argv[])
{
	srand((unsigned)time(NULL));
    typedef float T;
    typedef float RW;
    typedef VECTOR<T,3> TV;
    STREAM_TYPE stream_type((RW()));
    pthread_t driver_thread;
	rigids_example=new STRO_NEW<T>(stream_type);
    rigids_example->want_mpi_world=true;
    rigids_example->write_output_files=false;
    int level = 0;
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
    rigids_example->level = level;
    rigids_example->Parse(argc,argv);

#ifdef USE_MPI
    if (rigids_example->mpi_world->initialized) {
        rigids_example->mpi_rigids=new MPI_RIGIDS<TV>();
    }
        //rigids_example->Adjust_Output_Directory_For_MPI(rigids_example->mpi_rigids);}
#endif

    visualizer=new INTERACTIVE_VISUALIZATION<T,RW>();
    //INTERACTIVE_VISUALIZATION<T, RW> *visualizer=new INTERACTIVE_VISUALIZATION<T,RW>(rigids_example);
    visualizer->argc=argc;
    visualizer->argv=argv;
    visualizer->is_interactive = true; // TODO make an option later
    visualizer->opengl_world.Set_Idle_Callback(new OPENGL_CALLBACK_REDISPLAY(visualizer->opengl_world),0);
    //visualizer->data_lock=new pthread_mutex_t;
    //pthread_mutex_init(visualizer->data_lock,0);
    visualizer->opengl_world.animated_visualization = visualizer;

    driver = new SOLIDS_FLUIDS_INTERACTIVE_DRIVER_UNIFORM<GRID<TV> >(*rigids_example);
    rigids_example->driver = driver;

    driver->visualizer=visualizer;
    driver->Initialize();
	
/*	driver.rigid_component->rigid_geometry_collection_rendering1 = new RIGID_GEOMETRY_COLLECTION<TV>(*new RIGID_GEOMETRY_PARTICLES<TV>(),0);
	driver.rigid_component->rigid_geometry_collection_rendering1->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/jeepbody4", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering1->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backlefttire", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering1->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backrighttire", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering1->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontlefttire", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering1->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontrighttire", 1., true, false, false, false);
	driver.rigid_component->relative_frame_rendering1.Resize(5);
	FRAME<TV> F_ai1(driver.rigid_component->rigid_geometry_collection_simulation->particles.X(1), driver.rigid_component->rigid_geometry_collection_simulation->particles.rotation(1));
	FRAME<TV> F_ai2(driver.rigid_component->rigid_geometry_collection_simulation->particles.X(2), driver.rigid_component->rigid_geometry_collection_simulation->particles.rotation(2));
	FRAME<TV> F_ai3(driver.rigid_component->rigid_geometry_collection_simulation->particles.X(3), driver.rigid_component->rigid_geometry_collection_simulation->particles.rotation(3));
	FRAME<TV> F_ai4(driver.rigid_component->rigid_geometry_collection_simulation->particles.X(4), driver.rigid_component->rigid_geometry_collection_simulation->particles.rotation(4));
	FRAME<TV> F_ai5(driver.rigid_component->rigid_geometry_collection_simulation->particles.X(5), driver.rigid_component->rigid_geometry_collection_simulation->particles.rotation(5));
	FRAME<TV> F_ri1(TV(-0.0084, -0.2382+rigids_example->y_offset, 0.04052), ROTATION<TV>(2*acos(1), TV(-0.00082, -0.0011, 0.002)));
	FRAME<TV> F_ri2(TV(2.5451, -1.8074 + rigids_example->y_offset, -3.0132), ROTATION<TV>(2*acos(1),TV(0, 0, 0)));
	FRAME<TV> F_ri3(TV(-2.57365, -1.8074 + rigids_example->y_offset, -3.0132), ROTATION<TV>(2*acos(1),TV(0, 0, 0)));
	FRAME<TV> F_ri4(TV(2.5586, -1.8074 + rigids_example->y_offset, 4.15628), ROTATION<TV>(2*acos(1),TV(0, 0, 0)));
	FRAME<TV> F_ri5(TV(-2.5457, -1.8074 + rigids_example->y_offset, 4.15504), ROTATION<TV>(2*acos(1),TV(0, 0, 0)));
	
	driver.rigid_component->relative_frame_rendering1(1) = F_ai1.Inverse_Times(F_ri1);
	driver.rigid_component->relative_frame_rendering1(2) = F_ai2.Inverse_Times(F_ri2);
	driver.rigid_component->relative_frame_rendering1(3) = F_ai3.Inverse_Times(F_ri3);
	driver.rigid_component->relative_frame_rendering1(4) = F_ai4.Inverse_Times(F_ri4);
	driver.rigid_component->relative_frame_rendering1(5) = F_ai5.Inverse_Times(F_ri5);


	int size_mainbodies = 28;
	driver.rigid_component->rigid_geometry_collection_rendering0 = new RIGID_GEOMETRY_COLLECTION<TV>(*new RIGID_GEOMETRY_PARTICLES<TV>(),0);
	//attached to jeep body
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/jeepbody", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/base", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/insidegloss", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/controls", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/floor", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontpane", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/topframe", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/fenders", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/chairs", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/grille", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/leftsidemirror", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/leftsidemirrorhold", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/rightsidemirror", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/rightsidemirrorhold", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/leftwiper", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/rightwiper", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/sidegrille", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backgrille", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/toplightholds", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontsidelightholds", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backlightholds", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/steering", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backtire", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/toplights", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontlights", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backlights", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backsmalllights", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontsidelights", 1., true, false, false, false);
	//driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/windshield", 1., true, false, false, false);
	//attached to the tires
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backlefttire", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backrighttire", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontlefttire", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontrighttire", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backleftspoke", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/backrightspoke", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontleftspoke", 1., true, false, false, false);
	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/frontrightspoke", 1., true, false, false, false);
//	driver.rigid_component->rigid_geometry_collection_rendering0->Add_Rigid_Geometry(rigids_example->stream_type, "/data/Army2014/PhysBAM-HM/Public_Data/Archives/Rigid_Bodies/", 1., true, false, false, false);
	driver.rigid_component->relative_frame_rendering0.Resize(size_mainbodies+8);
	for(int i=1; i<=size_mainbodies; i++)
	{
		FRAME<TV> F_ai1(driver.rigid_component->rigid_geometry_collection_simulation->particles.X(1), driver.rigid_component->rigid_geometry_collection_simulation->particles.rotation(1));
		FRAME<TV> F_ri(driver.rigid_component->rigid_geometry_collection_rendering0->particles.X(i)+TV(0, rigids_example->y_offset, 0), driver.rigid_component->rigid_geometry_collection_rendering0->particles.rotation(i));
		driver.rigid_component->relative_frame_rendering0(i) = F_ai1.Inverse_Times(F_ri);
	}
	for(int i=size_mainbodies+1; i<=size_mainbodies+4; i++)
	{
		FRAME<TV> F_ai1(driver.rigid_component->rigid_geometry_collection_simulation->particles.X(i-size_mainbodies+1), driver.rigid_component->rigid_geometry_collection_simulation->particles.rotation(i-size_mainbodies+1));
		FRAME<TV> F_ri(driver.rigid_component->rigid_geometry_collection_rendering0->particles.X(i)+TV(0, rigids_example->y_offset, 0), driver.rigid_component->rigid_geometry_collection_rendering0->particles.rotation(i));
		driver.rigid_component->relative_frame_rendering0(i) = F_ai1.Inverse_Times(F_ri);
	}
	for(int i=size_mainbodies+5; i<=size_mainbodies+8; i++)
	{
		FRAME<TV> F_ai1(driver.rigid_component->rigid_geometry_collection_simulation->particles.X(i-size_mainbodies-4+1), driver.rigid_component->rigid_geometry_collection_simulation->particles.rotation(i-size_mainbodies-4+1));
		FRAME<TV> F_ri(driver.rigid_component->rigid_geometry_collection_rendering0->particles.X(i)+TV(0, rigids_example->y_offset, 0), driver.rigid_component->rigid_geometry_collection_rendering0->particles.rotation(i));
		driver.rigid_component->relative_frame_rendering0(i) = F_ai1.Inverse_Times(F_ri);
	}
	double f = 6000;
	driver.rigid_component->rect_front.Set_Vertex(VECTOR<double, 3>(f/2, -f/2, f/2), VECTOR<double, 3>(f/2, f/2, f/2), VECTOR<double, 3>(-f/2, -f/2, f/2), VECTOR<double, 3>(-f/2, f/2, f/2));
	driver.rigid_component->rect_front.Load_Texture("front.jpg");
	driver.rigid_component->rect_back.Set_Vertex(VECTOR<double, 3>(-f/2, -f/2, -f/2), VECTOR<double, 3>(-f/2, f/2, -f/2), VECTOR<double, 3>(f/2, -f/2, -f/2), VECTOR<double, 3>(f/2, f/2, -f/2));
	driver.rigid_component->rect_back.Load_Texture("back.jpg");
	driver.rigid_component->rect_left.Set_Vertex(VECTOR<double, 3>(f/2, -f/2, -f/2), VECTOR<double, 3>(f/2, f/2, -f/2), VECTOR<double, 3>(f/2, -f/2, f/2), VECTOR<double, 3>(f/2, f/2, f/2));
	driver.rigid_component->rect_left.Load_Texture("left.jpg");
	driver.rigid_component->rect_right.Set_Vertex(VECTOR<double, 3>(-f/2, -f/2, f/2), VECTOR<double, 3>(-f/2, f/2, f/2), VECTOR<double, 3>(-f/2, -f/2, -f/2), VECTOR<double, 3>(-f/2, f/2, -f/2));
	driver.rigid_component->rect_right.Load_Texture("right.jpg");
	driver.rigid_component->rect_top.Set_Vertex(VECTOR<double, 3>(f/2, f/2, f/2), VECTOR<double, 3>(f/2, f/2, -f/2), VECTOR<double, 3>(-f/2, f/2, f/2), VECTOR<double, 3>(-f/2, f/2, -f/2));
	driver.rigid_component->rect_top.Load_Texture("top.jpg");
*/
	visualizer->speed = &(rigids_example->curr_speed);
	visualizer->reversing = &(rigids_example->reverse);
	visualizer->Goto_Start_Frame();
    visualizer->PostInitialize_OpenGL_World();
    visualizer->Initialize_Lights();
	rigids_example->solids_evolution->rigid_body_collisions->collision_manager = rigids_example->collision_manager;

    pthread_create(&driver_thread, NULL, Execute_Driver, (void*) driver);
    Visualize((void*) visualizer);

    // Cancel the other thread and wait for it to finish.
    pthread_cancel(driver_thread);
    pthread_join(driver_thread, NULL);

    delete rigids_example;
    return 0;
}
//#####################################################################
