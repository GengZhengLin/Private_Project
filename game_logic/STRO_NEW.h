#ifndef __STRO_NEW__
#define __STRO_NEW__

#include <PhysBAM_Tools/Grids_Uniform/GRID.h>
#include <PhysBAM_Tools/Grids_Uniform/UNIFORM_GRID_ITERATOR_NODE.h>
#include <PhysBAM_Tools/Interpolation/INTERPOLATION_CURVE.h>
#include <PhysBAM_Tools/Log/DEBUG_PRINT.h>
#include <PhysBAM_Tools/Log/DEBUG_UTILITIES.h>
#include <PhysBAM_Tools/Parsing/PARSE_ARGS.h>
#include <PhysBAM_Tools/Random_Numbers/RANDOM_NUMBERS.h>
#include <PhysBAM_Geometry/Basic_Geometry/SPHERE.h>
#include <PhysBAM_Geometry/Collision_Detection/COLLISION_GEOMETRY_SPATIAL_PARTITION.h>
#include <PhysBAM_Geometry/Collisions/COLLISION_GEOMETRY_COLLECTION.h>
#include <PhysBAM_Geometry/Collisions/RIGID_COLLISION_GEOMETRY_3D.h>
#include <PhysBAM_Geometry/Implicit_Objects/ANALYTIC_IMPLICIT_OBJECT.h>
#include <PhysBAM_Geometry/Implicit_Objects/IMPLICIT_OBJECT_TRANSFORMED.h>
#include <PhysBAM_Geometry/Read_Write/Grids_Uniform_Level_Sets/READ_WRITE_LEVELSET_3D.h>
#include <PhysBAM_Geometry/Tessellation/RANGE_TESSELLATION.h>
#include <PhysBAM_Geometry/Tessellation/SPHERE_TESSELLATION.h>
#include <PhysBAM_Geometry/Topology_Based_Geometry/FREE_PARTICLES.h>
#include <PhysBAM_Geometry/Topology_Based_Geometry/STRUCTURE_LIST.h>
#include <PhysBAM_Geometry/Topology_Based_Geometry/TRIANGULATED_SURFACE.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Collisions/RIGID_BODY_COLLISIONS.h>
//#include <PhysBAM_Solids/PhysBAM_Rigids/Forces_And_Torques/RIGID_ETHER_DRAG.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Forces_And_Torques/RIGID_GRAVITY.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Rigid_Bodies/KINEMATIC_COLLISION_BODY.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Rigid_Bodies/RIGID_BODY.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Rigid_Bodies/RIGID_BODY_EVOLUTION_PARAMETERS.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Rigid_Bodies/RIGIDS_PARAMETERS.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Rigid_Bodies/RIGID_BODY_COLLISION_PARAMETERS.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Rigids_Evolution/RIGIDS_EVOLUTION.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Standard_Tests/RIGIDS_STANDARD_TESTS.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/RIGIDS_EXAMPLE.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Rigid_Body_Clusters/RIGID_BODY_CLUSTER_BINDINGS.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Rigid_Body_Clusters/RIGID_BODY_CLUSTER_BINDINGS_SIMPLE_FRACTURE.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Joints/JOINT.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Joints/PRISMATIC_TWIST_JOINT.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Joints/POINT_JOINT.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Joints/PRISMATIC_POINT_JOINT.h>
#include <PhysBAM_Solids/PhysBAM_Solids/Standard_Tests/SOLIDS_STANDARD_TESTS.h>
#include <PhysBAM_Solids/PhysBAM_Solids/Solids/SOLID_BODY_COLLECTION.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Joints/RIGID_JOINT.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Articulated_Rigid_Bodies/ARTICULATED_RIGID_BODY_3D.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Joints/JOINT_MESH.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Articulated_Rigid_Bodies/ARTICULATED_RIGID_BODY_IMPULSE_ACCUMULATOR.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Collisions/RIGID_BODY_IMPULSE_ACCUMULATOR.cpp>
#include <PhysBAM_Solids/PhysBAM_Solids/Solids/SOLIDS_PARAMETERS.h>
#include <fstream>
#include <PhysBAM_Solids/PhysBAM_Solids/Solids_Evolution/NEWMARK_EVOLUTION.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Collisions/RIGID_BODY_COLLISION_MANAGER_HASH.h>
#include <PhysBAM_Geometry/Basic_Geometry/HEIGHT_FIELD.h>

extern void initVisualizer();
extern VECTOR<float,3> getCameraDirection();
extern void setCameraShakeMagnitude(float);
extern int need_fog;
namespace PhysBAM{

template<class T_input>
class STRO_NEW:public SOLIDS_FLUIDS_EXAMPLE_UNIFORM<GRID<VECTOR<T_input,3> > >
{
    typedef T_input T;typedef VECTOR<T_input,3> TV;
    typedef enum BRIDGE_TYPE { PLANKS, RUBBER, SPRING } BRIDGE_TYPE;
public:
    SOLIDS_STANDARD_TESTS<TV> tests;
	SOLIDS_FLUIDS_INTERACTIVE_DRIVER_UNIFORM<GRID<TV> > * driver;
	RIGID_BODY_COLLISION_MANAGER_HASH* collision_manager;

    typedef SOLIDS_FLUIDS_EXAMPLE_UNIFORM<GRID<VECTOR<T_input,3> > > BASE;
    using BASE::solids_parameters;using BASE::fluids_parameters;using BASE::solid_body_collection;using BASE::solids_evolution;using BASE::test_number;
    using BASE::data_directory;using BASE::last_frame;using BASE::output_directory;using BASE::stream_type;using BASE::parse_args;
    bool firstRenderModel;
   	int condition;
   	int num_explosions;
   	bool need_explosions;
	T y_offset;
	T wheels_offset;
	T sphere_offset;
	T tireFriction;
	T springConst;
	T dampConst;
	T prismaticMin;	//min limit the spring can move in y space
	T prismaticMax;	//max limit the spring can move in y space
	T jeepX;
	T jeepY;
	T jeepZ;
	T height; // the initial height of the jeep off the ground
	TV wind_direction;
	TV wind_acceleration;
	int level;
	int is_shaking;
	int explosion_ptr;
	int mine_explosion_frame; // the frame on which the mine was hit
	bool blownUp; // tracker variable for being hit by a landmine; deactivate some functions when this is true

    STRO_NEW(const STREAM_TYPE stream_type)
        :BASE(stream_type,0,fluids_parameters.NONE),tests(*this,solid_body_collection)
    {
    	need_explosions = false;
    	explosion_ptr = 0;
    	num_explosions = 5;
    	explosions = (RIGID_GEOMETRY<TV>**)malloc(sizeof(RIGID_GEOMETRY<TV>*) * num_explosions);
    	height = (T)3;
    	wind_direction = TV(0.1f, 0.0f, 0.9f);
    	wind_direction.Normalize();
    	wind_acceleration = TV(0,0,0);
        condition = 0;
        level = 6;
        y_offset=2.98f;
		tireFriction = 1.0;
		springConst = 400;
		dampConst = 10;
		prismaticMin = -100.0f;	//min limit the spring can move in y space
		prismaticMax = -0.5f;	//max limit the spring can move in y space
        wheels_offset = 0.6f;
        sphere_offset = 0.02f;
		jeepX = 0.0;
		jeepY = 0.0;
		jeepZ = 0.0;
		blownUp = false;
		is_shaking = 0;
		car_flag = 0;
		mine_explosion_frame = -1;
        LOG::Initialize_Logging(true,true,0,true);        
    }

    ~STRO_NEW()
    {
    	//free(explosions);
    }

   	//virtual void Preprocess_Frame(const int frame){};
    //virtual void Postprocess_Frame(const int frame){};
    virtual void Preprocess_Substep(const T dt,const T time){};
    //virtual void Postprocess_Substep(const T dt,const T time){};


   	void Preprocess_Frame(const int frame)
	{
		//speed_limit = 0;
	}



	void Postprocess_Substep(const T dt,const T time)
	{
			/*
		if(forward && !back) Set_Speed(speed_limit);
		else Set_Speed(0.);
		*/

	}
    
    void Postprocess_Frame(const int frame)
	{
		/*
		if(dst_wise == 0) dst_angle = 0;
		else if(dst_wise > 0) dst_angle = pi/4.;
		else if(dst_wise < 0) dst_angle = -pi/4.;
		Turn_Wheels(dst_angle, pi/36./5.);
		*/
		if((wheels_angle>=pi/18 && dst_wise<0) || (wheels_angle<=-pi/18 && dst_wise>0)) Turn_Wheels(0., pi/18.);
		else Turn_Wheels(pi/18.);
		if (lift && !nitro){
			backleftsj->rest_length = 1.5;
			backrightsj->rest_length = 1.5;
			frontleftsj->rest_length = 1.5;
			frontrightsj->rest_length = 1.5;
		} else {
			backleftsj->rest_length = 0;
			backrightsj->rest_length = 0;
			frontleftsj->rest_length = 0;
			frontrightsj->rest_length = 0;
		}
		/*if(forward && !back) Accelerate(0.5, 10.);
		else if(!forward && back) Break(1.);*/
		if (reverse){ //TODO CALLBACKS: fix this part.
			Set_Speed(0.5 * -curr_accel); 
			nitro = false;
		} else {
			Set_Speed(0.5 * curr_accel);
			Nitro(frame);
		}
		//forward = false;
		//back = false;
		curr_speed = Get_Speed();
		float s = 2;
		for (int i = 1; i <= rocks.Size(); ++i) {
			if (rocks(i)->X()(2) < -50) {
				rocks(i)->X() = TV(125 * s + (-0.5+rand() / (float)RAND_MAX) * 30.0, 400, 125 * s + (-0.5+rand() / (float)RAND_MAX) * 30.0);
				rocks(i)->V() = TV(0, 0, 0);
			}
		}

		wind_acceleration += TV(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float) RAND_MAX);
		wind_acceleration *= exp(-wind_acceleration.Magnitude_Squared());
		wind_direction += wind_acceleration * 0.1;
		wind_direction.Normalize();

		//////////////////////////////////////////////////////////////////////
		// Rotate the car flag in the wind
		//////////////////////////////////////////////////////////////////////
		if (car_flag) {
			car_flag->X() = jeep->X() + jeep->Frame().r.Rotation_Matrix() * TV(0, 6.75, -1);

			TV y_axis = TV(0, 1, 0);
			TV z_axis = TV(0, 0, 1);
			TV pole_axis = jeep->Frame().r.Rotation_Matrix() * y_axis;
			TV normal = TV::Cross_Product(pole_axis, wind_direction);
			TV roll1 = TV::Cross_Product(y_axis, pole_axis);
			T angle1 = TV::Angle_Between(y_axis, pole_axis);
			ROTATION<TV> r1 = ROTATION<TV>(angle1, roll1);
			z_axis = r1.Rotation_Matrix() * z_axis;
			TV roll2 = TV::Cross_Product(z_axis, normal);
			T angle2 = TV::Angle_Between(z_axis, normal);
			ROTATION<TV> r2 = ROTATION<TV>(angle2, roll2);
			car_flag->Rotation() = r2 * r1;
		}

		//////////////////////////////////////////////////////////////////////
		// Rotate the goal flag in the wind
		//////////////////////////////////////////////////////////////////////
		if (flag) {
			TV y_axis = TV(0, 1, 0);
			TV z_axis = TV(0, 0, 1);
			TV normal = TV::Cross_Product(wind_direction, y_axis);
			flag->Rotation() = ROTATION<TV>(pi-TV::Angle_Between(z_axis, normal), y_axis);
		}

		//////////////////////////////////////////////////////////////////////
		// Stop the explosion from looping
		//////////////////////////////////////////////////////////////////////
		if (need_explosions) {
			if (explosions[explosion_ptr]->start_play &&
				explosions[explosion_ptr]->frame_number > explosions[explosion_ptr]->simplicial_object->textures.m) {
				// The explosion has looped around.
				explosions[explosion_ptr]->start_play = 0;
			}
		}
		//////////////////////////////////////////////////////////////////////
		// Add camera shake
		//////////////////////////////////////////////////////////////////////
		if (blownUp && mine_explosion_frame == -1) {
			// Set it for the first time.
			mine_explosion_frame = frame;
		}

		if (mine_explosion_frame == -1) {
			setCameraShakeMagnitude(0);
			is_shaking = 0;
		} else {
			float sigma = 25.0;
			float val = exp(-pow(frame - mine_explosion_frame, 2) / pow(sigma, 2));
			is_shaking = 1;
			if (val < 0.1) {
				val = 0;
				is_shaking = 0;
			}
			setCameraShakeMagnitude(val);
		}
	}
	
	void Post_Initialization()
    {
        /*solids_parameters.rigid_body_collision_parameters.use_analytic_collisions=true;
        RIGID_BODY_COLLISIONS<TV>& collisions=*rigids_evolution->rigid_body_collisions;
        collisions.Register_Analytic_Collisions();*/
    }

	void Nitro(const int frame)
	{
		if (blownUp) {
			return;
		}

		if(nitro)
		{
			trig_nitro = frame;
			nitro = false;
		}

		const int keep = 24;//24*3;
		if(frame >= trig_nitro && frame <= trig_nitro+keep)
		{
			Accelerate_Nitro(2., 30.);
		}
	}

	bool checkTireOrJeep(const char* name) {
		if (std::string(name) == "jeep") {
			return true;
		}
		int len = strlen(name);
		return len >= 4 && strcmp(name + len - 4, "tire") == 0;
	}

	void move_jeep(const TV& velocity) {
		jeep->V() = velocity;
		backlefttire->V() = velocity;
		backrighttire->V() = velocity;
		frontlefttire->V() = velocity;
		frontrighttire->V() = velocity;
	}

	void destroy_jeep() {
//		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Remove_All();
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Deactivate_Articulation(backleftsj->id_number);
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Deactivate_Articulation(backrightsj->id_number);
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Deactivate_Articulation(frontleftsj->id_number);
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Deactivate_Articulation(frontrightsj->id_number);
	}

	void Setup_Explosions() {
		for (int i = 0; i < num_explosions; ++i) {
			explosions[i] = pushRenderingModel(
				stream_type,
				std::string("C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/explosion"),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(1,1,1), 1.)),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(1,1,1), 1.)),
				-1,
				TV(0, 0, 0),
				false,
				3,
				true, false, false, false, i == 0);
			explosions[i]->simplicial_object->textures = explosions[0]->simplicial_object->textures;
			explosions[i]->omit_simulation_binding = true;
			explosions[i]->mode = 1;
		}
	}


	void Hit_Landmine() {
		const TV& velocity = TV((rand() / (float)RAND_MAX - 0.5) * 25,(rand() / (float)RAND_MAX) * 5 + 15, (rand() / (float)RAND_MAX - 0.5) * 25);
		if (!blownUp) {
			// first hit; move jeep and wheels
			condition = 3;
			move_jeep(velocity);
			destroy_jeep();
			Set_Speed(0);
			blownUp = true;
		} else {
			// subsequent hits; only move the jeep
			jeep->V() = velocity;
		}
	}

	void process_collisions(RIGID_BODY<TV>& body1, RIGID_BODY<TV>& body2) {
		if ((checkTireOrJeep(body1.name.c_str()) && body2.name == "mine") ||
			(body1.name == "mine" && checkTireOrJeep(body2.name.c_str()))) {
			RIGID_BODY<TV>* mine = (body1.name == "mine") ? &body1 : &body2;
			RIGID_BODY<TV>* other = (mine == &body1) ? &body2 : &body1;
			//mine has exploded
			if (mine->triggered)
				return;

			if (need_explosions) {
				if (explosions[explosion_ptr]->start_play && 
					explosions[explosion_ptr]->frame_number <= explosions[explosion_ptr]->simplicial_object->textures.m)
					return;
				mine->triggered = 1;
				explosions[explosion_ptr]->start_play = 1;
				explosions[explosion_ptr]->frame_number = 0;
				explosions[explosion_ptr]->X() = mine->X() + TV(0, -5, 0);
				TV billboard_normal = (TV)getCameraDirection();
				billboard_normal(2) = 0;
				explosions[explosion_ptr]->Rotation() = ROTATION<TV>(TV::Angle_Between(TV(0, 0, 1), billboard_normal), TV(0,1,0));
				explosion_ptr = (explosion_ptr + 1) % num_explosions;
			}
			if (!blownUp || other->name == "jeep") {
				Hit_Landmine();
				mine_explosion_frame = -1;
			}
		}
		if ((body1.name == "jeep" && body2.name == "flagpole") ||
			(body1.name == "flagpole" && body2.name == "jeep")) {
			condition = 1;
		}
		if (level == 2) {
			if ((body1.name == "ball" && body2.name == "button") ||
				(body1.name == "button" && body2.name == "ball")) {
				gatest->Use_Phi_Constraint(-pi, pi);
			}
		}
	}


	TV Get_Velocity()
	{
		TV flvel = frontlefttire->Rotation().Inverse_Rotate(frontlefttire->Angular_Velocity());
		TV frvel = frontrighttire->Rotation().Inverse_Rotate(frontrighttire->Angular_Velocity());
		TV blvel = backlefttire->Rotation().Inverse_Rotate(backlefttire->Angular_Velocity());
		TV brvel = backrighttire->Rotation().Inverse_Rotate(backrighttire->Angular_Velocity());
			
		return (flvel+frvel+blvel+brvel)/4; 
	}

	T Get_Speed()
	{
		return Get_Velocity()(3);	//???
	}

	void Set_Speed(const T input_speed)
	{
		if (blownUp) {
			return;
		}

			/*
		frontlefttire->Angular_Velocity()=TV(input_speed,0.,0.);
	    frontlefttire->Update_Angular_Momentum();
		frontrighttire->Angular_Velocity()=TV(input_speed,0.,0.);
	    frontrighttire->Update_Angular_Momentum();
		backlefttire->Angular_Velocity()=TV(input_speed,0.,0.);
	    backlefttire->Update_Angular_Momentum();
		backrighttire->Angular_Velocity()=TV(input_speed,0.,0.);
	    backrighttire->Update_Angular_Momentum();
*/

		frontlefttire->Angular_Velocity()=frontlefttire->Rotation().Rotate(TV(0.,0.,input_speed));
	    frontlefttire->Update_Angular_Momentum();
		frontrighttire->Angular_Velocity()=frontrighttire->Rotation().Rotate(TV(0.,0.,input_speed));
	    frontrighttire->Update_Angular_Momentum();
		backlefttire->Angular_Velocity()=backlefttire->Rotation().Rotate(TV(0.,0.,input_speed));
	    backlefttire->Update_Angular_Momentum();
		backrighttire->Angular_Velocity()=backrighttire->Rotation().Rotate(TV(0.,0.,input_speed));
	    backrighttire->Update_Angular_Momentum();
	}

	void Accelerate(const T dv, const T sl)
	{
		//linear acceleration
		T v_cur = Get_Speed();
		if(sl > 0. && v_cur < sl && v_cur > -sl)
		{
			if (!reverse) {Set_Speed(v_cur + dv);} else {
				if (v_cur > 0) {Set_Speed(0);} else{
				Set_Speed(v_cur - dv); }
			}
		}
		/*else if(sl < 0. && (v_cur > sl || v_cur < -sl))
		{
			if (!reverse) {Set_Speed(v_cur - dv);} else {Set_Speed();}
		}*/
		/*
		const T dv = 0.3;//20/3/24;
		const T speed_limit_back = 5;
		T v_cur = Get_Speed();

		if(forward && v_cur < speed_limit)
		{
			Set_Speed(v_cur + dv);
		}
		else if(back && v_cur > speed_limit_back)
		{
			Set_Speed(v_cur - dv);
		}
		*/
	}

	// for the confused: this should be Brake :/
	void Break(const T dv)
	{
		if (blownUp) {
			return;
		}

		T v_cur = Get_Speed();
		//const T dv = 1.;
		if(v_cur >= dv)
		{
			Set_Speed(v_cur - dv);
		}
		else if(v_cur <= -dv)
		{
			Set_Speed(v_cur + dv);
		}
		else
		{
			Set_Speed(0);
		}
	}

	void Accelerate_Nitro(const T dv, const T sl)
	{
		T v_cur = curr_speed;
		if(sl > 0. && v_cur < sl)
		{
			Set_Speed(v_cur + dv);
			jeep->V() = jeep->Rotation().Rotate(TV(0., 0., v_cur + dv));
		}
		else if(sl < 0. && v_cur > sl)
		{
			Set_Speed(v_cur - dv);
			jeep->V() = jeep->Rotation().Rotate(TV(0., 0., v_cur - dv));
		}
	}

	void Set_Wheels(const T angle)
	{
		//j_jb_fls->Use_Phi_Constraint(angle, angle);
		//j_jb_frs->Use_Phi_Constraint(angle, angle);
		frontleftsj->Use_Phi_Constraint(angle, angle);
		frontrightsj->Use_Phi_Constraint(angle, angle);
		wheels_angle = angle;
	}
	
	void Turn_Wheels(const T dt, const T time, T &trig, const T turn, const T step = pi/36.)
	{
		if (blownUp) {
			return;
		}

		if(time >= trig)
		{
			//printf("END Frame time = %f", time);
			const T range = pi/4.;	
			
			T dest;
			int wise;
			T dangle = turn - wheels_angle;
			if(dangle > -step && dangle <step) wise = 0;
			else if(dangle > 0) wise = 1;
			else wise = -1;
		
			
			if(wise != 0)
			{
				//
			
				dest = wheels_angle + wise * step;
				if(dest >= -range && dest <= range)
				{
					Set_Wheels(dest);
				}
			}
		}
	}

	void Turn_Wheels(const T angle_dst, const T step)
	{
		if (blownUp) {
			return;
		}

		//printf("END Frame time = %f", time);
		const T range = pi/6.;	
		
		T dest;
		int wise;
		T dangle = angle_dst - wheels_angle;
		if(dangle > -step && dangle <step) wise = 0;
		else if(dangle > 0) wise = 1;
		else wise = -1;
	
		
		if(wise != 0)
		{
			//
		
			dest = wheels_angle + wise * step;
			if(dest >= -range && dest <= range)
			{
				Set_Wheels(dest);
			}
		}
	}

	void Turn_Wheels(const T step)
	{
		if (blownUp) {
			return;
		}

		//printf("END Frame time = %f", time);
		const T range = pi/6.;	
		
		int wise = dst_wise;
		T dest;
	
		
		if(wise != 0)
		{
			//
		
			dest = wheels_angle + wise * step;
			if(dest >= -range && dest <= range)
			{
				Set_Wheels(dest);
			}
		}
		
		dst_wise = 0;
	}



    void Initialize_Bodies() PHYSBAM_OVERRIDE
    {
    	initVisualizer();
        solids_parameters.rigid_body_evolution_parameters.simulate_rigid_bodies=true;
        solids_parameters.cfl=1;
        solids_parameters.rigid_body_evolution_parameters.correct_evolution_energy=true;
        T skybox_scale = 1e4;

		Jeep(0);

		RIGID_BODY<TV>* skybox = &tests.Add_Rigid_Body("skybox", skybox_scale, (T)0.1);
		skybox->X() = TV(0, 0, 0);
		skybox->is_static = true;
		skybox->Set_Name("skybox");
	    pushSimulationModel(skybox);
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/skybox", 
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(1,1,1), 1.)),
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(1,1,1), 1.)),
	    		-1, TV(0, height, 0),false, skybox_scale);
		//Load_Obstacles("obstacles");
		collision_manager=new RIGID_BODY_COLLISION_MANAGER_HASH;
//		collision_manager->default_collide = false;
		switch (level) {
		case 0:
			Scene0();
			break;
		case 1:
			Scene1();
			break;
		case 2:
			Scene2();
			break;
		case 3:
			Scene3();
			break;
		case 4:
			Scene4();
			break;
		case 5:
			Scene5();
			break;
		case 6:
			Scene6();
			break;
		case 7:
			Scene7();
			break;
		default:
			//__android_log_print(ANDROID_LOG_WARN, "PhysBAM", "WARNING: unknown level %d; defaulting to level = 0", level);
//			Scene0();
			break;
		}
		if (need_fog)
			Add_Fog();

		driver->rigid_component->terrain = terrain_render;
		driver->rigid_component->ice = ground;
        solid_body_collection.rigid_body_collection.Add_Force(new RIGID_GRAVITY<TV>(solid_body_collection.rigid_body_collection, true));
    }
    
//#####################################################################

//jeep sim


	void add_jeep_body() {
		jeep = &tests.Add_Rigid_Body("approx_jeepbody", 1, (T) .1);	//simulate with the approximate model
		jeep->Set_Mass(30);
//		jeep->X() = TV(jeepX, jeepY + y_offset, jeepZ);
		jeep->X()=TV(-0.00994134, -0.24 + height, -0.0739949);
		jeep->Set_Coefficient_Of_Restitution(0.5);
		jeep->Set_Name("jeep");
	    pushSimulationModel(jeep);
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/jeepbody", 
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 20./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 20./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/insidegloss",
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(67./255., 34./255., 16./255.), 1.)),
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(67./255., 34./255., 16./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/controls",
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(157./255., 125./255., 83./255.), 1.)),
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(157./255., 125./255., 83./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/floor",
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(80./255., 67./255., 48./255.), 1.)),
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(80./255., 67./255., 48./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/frontpane",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/topframe",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/fenders",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/chairs",
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(137./255., 125./255., 103./255.), 1.)),
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(137./255., 125./255., 103./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/grille",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/leftsidemirror",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(210./255., 210./255., 210./255.), 0.95)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(210./255., 210./255., 210./255.), 0.95)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/leftsidemirrorhold",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/rightsidemirror",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(210./255., 210./255., 210./255.), 0.95)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(210./255., 210./255., 210./255.), 0.95)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/rightsidemirrorhold",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/leftwiper",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/rightwiper",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/sidegrille",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/backgrille",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/toplightholds",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/frontsidelightholds",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/backlightholds",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/steering",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/backtire",
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(60./255., 67./255., 73./255.), 1.)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/toplights",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(210./255., 210./255., 220./255.), 0.6)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(210./255., 210./255., 220./255.), 0.6)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/frontlights",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(210./255., 210./255., 220./255.), 0.6)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(210./255., 210./255., 220./255.), 0.6)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/backlights",
	    		OPENGL_MATERIAL::Plastic(OPENGL_COLOR(TV(200./255., 15./255., 60./255.), 0.6)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 15./255., 60./255.), 0.6)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/backsmalllights",
	    		OPENGL_MATERIAL::Plastic(OPENGL_COLOR(TV(250./255., 161./255., 5./255.), 0.6)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(250./255., 161./255., 5./255.), 0.6)),
	    		-1, TV(0, height, 0));
	    pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/frontsidelights",
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(250./255., 161./255., 5./255.), 0.6)),
	    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(250./255., 161./255., 5./255.), 0.6)),
	    		-1, TV(0, height, 0));

		car_pole = driver->rigid_component->rigid_geometry_collection_rendering->Add_Analytic_Cylinder((T) 6.5, (T) 0.1);
		car_pole->X() = jeep->X() + TV(0, 5.5, -1);
		car_pole->Rotation() = ROTATION<TV>(pi / 2, TV(1.,0.,0.));

		process_rendering(-1, OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1), 1.)),
			OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1), 1.)));

	    car_flag = pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/flag",
    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)),
    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)),
    		-1, TV(0, height, 0), false, 1.0f);
	    car_flag->omit_simulation_binding = true;
	}

	void add_jeep_wheels() {
		TV trans = TV(0.0 + wheels_offset, 0.0 + y_offset, 0.0);
		ROTATION<TV> rot = ROTATION<TV>(pi / 2., TV(0., 1., 0.));

		backlefttire = &tests.Add_Analytic_Cylinder(.7, (T) 1.1);
		backlefttire->X() = TV(2.55 + wheels_offset + jeepX, -1.8 + y_offset + jeepY, -3 + jeepZ);
		backlefttire->Rotation() = rot;
		backlefttire->Set_Mass(10);
		backlefttire->Set_Coefficient_Of_Restitution(0.0);
		backlefttire->Set_Coefficient_Of_Friction(tireFriction);
		backlefttire->Set_Name("backlefttire");
		backlefttire->Update_Angular_Momentum();
		backlefttire->impulse_accumulator = new RIGID_BODY_IMPULSE_ACCUMULATOR<
				TV, TV::dimension - 1>(*backlefttire);
		pushSimulationModel(backlefttire);
		pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/backlefttire",
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(50./255., 50./255., 50./255.), 1.)),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(50./255., 50./255., 50./255.), 1.)));
		int id = driver->rigid_component->next_for_rendering.Size();
		backrighttire = &tests.Add_Analytic_Cylinder(.7, (T) 1.1);
		backrighttire->X() = TV(-2.55 - wheels_offset + jeepX, -1.8 + y_offset + jeepY, -3 + jeepZ);
		backrighttire->Rotation() = rot;
		backrighttire->Set_Mass(10);
		backrighttire->Set_Coefficient_Of_Restitution(0.0);
		backrighttire->Set_Coefficient_Of_Friction(tireFriction);
		backrighttire->Set_Name("backrighttire");
		backrighttire->Update_Angular_Momentum();
		backrighttire->impulse_accumulator = new RIGID_BODY_IMPULSE_ACCUMULATOR<
				TV, TV::dimension - 1>(*backrighttire);
		pushSimulationModel(backrighttire);
		pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/backrighttire",
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(50./255., 50./255., 50./255.), 1.)),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(50./255., 50./255., 50./255.), 1.)));

		//front left
		frontlefttire = &tests.Add_Analytic_Cylinder(.7, (T) 1.1);
		frontlefttire->X() = TV(2.55 + wheels_offset + jeepX, -1.8 + y_offset + jeepY, 4.155 + jeepZ);
		frontlefttire->Rotation() = rot;
		frontlefttire->Set_Mass(10);
		frontlefttire->Set_Coefficient_Of_Restitution(0.0);
		frontlefttire->Set_Coefficient_Of_Friction(tireFriction);
		frontlefttire->Set_Name("frontlefttire");
		frontlefttire->Update_Angular_Momentum();
		frontlefttire->impulse_accumulator = new RIGID_BODY_IMPULSE_ACCUMULATOR<
				TV, TV::dimension - 1>(*frontlefttire);
		pushSimulationModel(frontlefttire);
		pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/frontlefttire",
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(50./255., 50./255., 50./255.), 1.)),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(50./255., 50./255., 50./255.), 1.)));

		//front right
		frontrighttire = &tests.Add_Analytic_Cylinder(.7, (T) 1.1);
		frontrighttire->X() = TV(-2.55 - wheels_offset + jeepX, -1.8 + y_offset + jeepY, 4.155 + jeepZ);
		frontrighttire->Rotation() = rot;
		frontrighttire->Set_Mass(10);
		frontrighttire->Set_Coefficient_Of_Restitution(0.0);
		frontrighttire->Set_Coefficient_Of_Friction(tireFriction);
		frontrighttire->Set_Name("frontrighttire");
		frontrighttire->Update_Angular_Momentum();
		frontrighttire->impulse_accumulator =
				new RIGID_BODY_IMPULSE_ACCUMULATOR<TV, TV::dimension - 1>(
						*frontrighttire);
		pushSimulationModel(frontrighttire);
		pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/frontrighttire",
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(50./255., 50./255., 50./255.), 1.)),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(50./255., 50./255., 50./255.), 1.)));
		backleftsphere = &tests.Add_Rigid_Body("sphere", 0.1, (T) .1);
		backleftsphere->X() = TV(2.55 + sphere_offset + jeepX, -1.8 + y_offset + jeepY, -3 + jeepZ);
		backleftsphere->Set_Mass(10);
		backleftsphere->Set_Coefficient_Of_Restitution(0.0);
		backleftsphere->Set_Name("backleftsphere");
		pushSimulationModel(backleftsphere);
		pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/backleftspoke",
				OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),
				OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),id);

		backrightsphere = &tests.Add_Rigid_Body("sphere", 0.1, (T) .1);
		backrightsphere->X() = TV(-2.55 - sphere_offset + jeepX, -1.8 + y_offset + jeepY, -3 + jeepZ);
		backrightsphere->Set_Mass(10);
		backrightsphere->Set_Coefficient_Of_Restitution(0.0);
		backrightsphere->Set_Name("backrightsphere");
		pushSimulationModel(backrightsphere);
		pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/backrightspoke",
				OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),
				OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),id+1);

		frontleftsphere = &tests.Add_Rigid_Body("sphere", 0.1, (T) .1);
		frontleftsphere->X() = TV(2.55 + sphere_offset + jeepX, -1.8 + y_offset + jeepY, 4.155 + jeepZ);
		frontleftsphere->Set_Mass(10);
		frontleftsphere->Set_Coefficient_Of_Restitution(0.0);
		frontleftsphere->Set_Name("frontleftsphere");
		pushSimulationModel(frontleftsphere);
		pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/frontleftspoke",
				OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),
				OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),id+2);

		frontrightsphere = &tests.Add_Rigid_Body("sphere", 0.1, (T) .1);
		frontrightsphere->X() = TV(-2.55 - sphere_offset + jeepX, -1.8 + y_offset + jeepY,	4.155 + jeepZ);
		frontrightsphere->Set_Mass(10);
		frontrightsphere->Set_Coefficient_Of_Restitution(0.0);
		frontrightsphere->Set_Name("frontrightsphere");
		pushSimulationModel(frontrightsphere);
		pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/frontrightspoke",
				OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),
				OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(200./255., 200./255., 200./255.), 1.)),id+3);

		/*
		 * Joints
		 */
		// Back Left
		backleftsj = new PRISMATIC_POINT_JOINT<TV>();
		//backleftsj->Use_Phi_Constraint(-pi/24.,pi/24.);
		backleftsj->Use_Phi_Constraint(0., 0.);
		backleftsj->Use_Theta_Constraint(0., 0.);
		backleftsj->Use_Twist_Constraint(0., 0.);
	    backleftsj->constrain.Fill(true);
	    backleftsj->prismatic_min=TV();
	    backleftsj->prismatic_max=TV();
	    backleftsj->prismatic_min(2)=prismaticMin;
	    backleftsj->prismatic_max(2)=prismaticMax;
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
				jeep->particle_index, backleftsphere->particle_index,backleftsj);
		backleftsj->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., 0.), ROTATION<TV>(0., TV(0., 1., 0.))));
		backleftsj->Set_Joint_To_Parent_Frame(
				jeep->Frame().Inverse_Times(backleftsphere->Frame()	* FRAME<TV>(TV(0., 0., 0.),	ROTATION<TV>(0., TV(0., 1., 0.)))));
		backleftsj->Add_Spring(jeep->Frame().Inverse_Times(backleftsphere->Frame()*TV(0.,0.,0.)), TV(0.,0.0,0),springConst,dampConst,0.0,10/3.);


		backleftst = new POINT_JOINT<TV>();
		backleftst->Use_Phi_Constraint(0., 0.);
		backleftst->Use_Twist_Constraint(0., 0.);

		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
				backleftsphere->particle_index, backlefttire->particle_index,backleftst);
		backleftst->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., -0.3156),ROTATION<TV>(0., TV(0., 1., 0.))));
		backleftst->Set_Joint_To_Parent_Frame(backleftsphere->Frame().Inverse_Times(
						backlefttire->Frame()* FRAME<TV>(TV(0., 0., -0.3156),ROTATION<TV>(0., TV(0., 1., 0.)))));

		// Back Right
		backrightsj = new PRISMATIC_POINT_JOINT<TV>();
		//backrightsj->Use_Phi_Constraint(-pi/24.,pi/24.);
		backrightsj->Use_Phi_Constraint(0., 0.);
		backrightsj->Use_Theta_Constraint(0., 0.);
		backrightsj->Use_Twist_Constraint(0., 0.);
	    backrightsj->constrain.Fill(true);
	    backrightsj->prismatic_min=TV();
	    backrightsj->prismatic_max=TV();
	    backrightsj->prismatic_min(2)=prismaticMin;
	    backrightsj->prismatic_max(2)=prismaticMax;
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
				jeep->particle_index, backrightsphere->particle_index,backrightsj);
		backrightsj->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., 0.), ROTATION<TV>(0., TV(0., 1., 0.))));
		backrightsj->Set_Joint_To_Parent_Frame(jeep->Frame().Inverse_Times(backrightsphere->Frame()
								* FRAME<TV>(TV(0., 0., 0.),	ROTATION<TV>(0., TV(0., 1., 0.)))));
		backrightsj->Add_Spring(jeep->Frame().Inverse_Times(backrightsphere->Frame()*TV(0.,0.,0.)), TV(0.,0.0,0),springConst,dampConst,0.0,10/3.);

		backrightst = new POINT_JOINT<TV>();
		backrightst->Use_Phi_Constraint(0., 0.);
		backrightst->Use_Twist_Constraint(0., 0.);
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
				backrightsphere->particle_index, backrighttire->particle_index,	backrightst);
		backrightst->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., 0.3156),	ROTATION<TV>(0., TV(0., 1., 0.))));
		backrightst->Set_Joint_To_Parent_Frame(	backrightsphere->Frame().Inverse_Times(	backrighttire->Frame()
								* FRAME<TV>(TV(0., 0., 0.3156),	ROTATION<TV>(0., TV(0., 1., 0.)))));

		// Front Left
		frontleftsj = new PRISMATIC_POINT_JOINT<TV>();
		//frontleftsj->Use_Phi_Constraint(-pi/24.,pi/24.);
		frontleftsj->Use_Phi_Constraint(0., 0.);
		frontleftsj->Use_Theta_Constraint(0., 0.);
		frontleftsj->Use_Twist_Constraint(0., 0.);
	    frontleftsj->constrain.Fill(true);
	    frontleftsj->prismatic_min=TV();
	    frontleftsj->prismatic_max=TV();
	    frontleftsj->prismatic_min(2)=prismaticMin;
	    frontleftsj->prismatic_max(2)=prismaticMax;
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
				jeep->particle_index, frontleftsphere->particle_index,frontleftsj);
		frontleftsj->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., 0.), ROTATION<TV>(0., TV(0., 1., 0.))));
		frontleftsj->Set_Joint_To_Parent_Frame(jeep->Frame().Inverse_Times(	frontleftsphere->Frame()
								* FRAME<TV>(TV(0., 0., 0.),	ROTATION<TV>(0., TV(0., 1., 0.)))));
		frontleftsj->Add_Spring(jeep->Frame().Inverse_Times(frontleftsphere->Frame()*TV(0.,0.,0.)), TV(0.,0.0,0),springConst,dampConst,0.0,10/3.);
		//initial_frame_front_left.t(2) += 1.5;

		frontleftst = new POINT_JOINT<TV>();
		frontleftst->Use_Phi_Constraint(0., 0.);
		frontleftst->Use_Twist_Constraint(0., 0.);
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
				frontleftsphere->particle_index, frontlefttire->particle_index,	frontleftst);
		frontleftst->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., -0.3156),ROTATION<TV>(0., TV(0., 1., 0.))));
		frontleftst->Set_Joint_To_Parent_Frame(frontleftsphere->Frame().Inverse_Times(
						frontlefttire->Frame()* FRAME<TV>(TV(0., 0., -0.3156),ROTATION<TV>(0., TV(0., 1., 0.)))));

		// Front Right
		frontrightsj = new PRISMATIC_POINT_JOINT<TV>();
		//frontrightsj->Use_Phi_Constraint(-pi/24.,pi/24.);
		frontrightsj->Use_Phi_Constraint(0., 0.);
		frontrightsj->Use_Theta_Constraint(0., 0.);
		frontrightsj->Use_Twist_Constraint(0., 0.);
	    frontrightsj->constrain.Fill(true);
	    frontrightsj->prismatic_min=TV();
	    frontrightsj->prismatic_max=TV();
	    frontrightsj->prismatic_min(2)=prismaticMin;
	    frontrightsj->prismatic_max(2)=prismaticMax;
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
				jeep->particle_index, frontrightsphere->particle_index,	frontrightsj);
		frontrightsj->Set_Joint_To_Child_Frame(	FRAME<TV>(TV(0., 0., 0.), ROTATION<TV>(0., TV(0., 1., 0.))));
		frontrightsj->Set_Joint_To_Parent_Frame(jeep->Frame().Inverse_Times(frontrightsphere->Frame()
								* FRAME<TV>(TV(0., 0., 0.),	ROTATION<TV>(0., TV(0., 1., 0.)))));
		frontrightsj->Add_Spring(jeep->Frame().Inverse_Times(frontrightsphere->Frame()*TV(0.,0.,0.)), TV(0.,0.0,0),springConst,dampConst,0.0,10/3.);

		frontrightst = new POINT_JOINT<TV>();
		frontrightst->Use_Phi_Constraint(0., 0.);
		frontrightst->Use_Twist_Constraint(0., 0.);
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
				frontrightsphere->particle_index,frontrighttire->particle_index, frontrightst);
		frontrightst->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., 0.3156),ROTATION<TV>(0., TV(0., 1., 0.))));
		frontrightst->Set_Joint_To_Parent_Frame(frontrightsphere->Frame().Inverse_Times(frontrighttire->Frame()
								* FRAME<TV>(TV(0.0f, 0.0f, 0.3156f),	ROTATION<TV>(0.0f, TV(0.0f, 1.0f, 0.0f)))));

		//solid_body_collection.rigid_body_collection.articulated_rigid_body.poststabilization_iterations = 100;
		solid_body_collection.rigid_body_collection.articulated_rigid_body.use_epsilon_scale = false;
		solid_body_collection.rigid_body_collection.articulated_rigid_body.iterative_tolerance = 1e-2f;	//TODO: Performance?
		//solid_body_collection.rigid_body_collection.articulated_rigid_body.use_poststab_in_cg=false;
	}


//add joint to child frame.
void set_joint2child(POINT_JOINT<TV>* &joint, RIGID_BODY<TV>* &parent, RIGID_BODY<TV>* &child, FRAME<TV> joint2childF)
{
	arb->joint_mesh.Add_Articulation(parent->particle_index, child->particle_index, joint);
    	joint->Set_Joint_To_Child_Frame(joint2childF);
    	joint->Set_Joint_To_Parent_Frame(parent->Frame().Inverse_Times(child->Frame()*joint2childF));	
}
void set_joint2child(PRISMATIC_TWIST_JOINT<TV>* &joint, RIGID_BODY<TV>* &parent, RIGID_BODY<TV>* &child, FRAME<TV> joint2childF)
{
	arb->joint_mesh.Add_Articulation(parent->particle_index, child->particle_index, joint);
    	joint->Set_Joint_To_Child_Frame(joint2childF);
    	joint->Set_Joint_To_Parent_Frame(parent->Frame().Inverse_Times(child->Frame()*joint2childF));	
}
void set_joint2child(RIGID_JOINT<TV>* &joint, RIGID_BODY<TV>* &parent, RIGID_BODY<TV>* &child, FRAME<TV> joint2childF)
{
	arb->joint_mesh.Add_Articulation(parent->particle_index, child->particle_index, joint);
    	joint->Set_Joint_To_Child_Frame(joint2childF);
    	joint->Set_Joint_To_Parent_Frame(parent->Frame().Inverse_Times(child->Frame()*joint2childF));	
}

void Initialize_Drive_Param()
{
	speed_limit = 10.;
	forward = false;
	back = false;
	dst_wise = 0;
	wheels_angle = 0.;
	dst_angle = 0.;
	nitro = false;
	trig_nitro = -999;
	reverse = false;
	reverse_ref = false;
	curr_speed = 0.;
	curr_accel = 0;
	lift = false;
}

void Jeep(const T angle){
    Initialize_Drive_Param();
	arb = &solid_body_collection.rigid_body_collection.articulated_rigid_body;
	add_jeep_body();
    add_jeep_wheels();

//	terrain = &tests.Add_Height_Field("C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/height_map.jpg", TV(3000, 200, 3000));
//	pushSimulationModel(terrain);

    solids_parameters.rigid_body_collision_parameters.use_legacy_push_out=false;
    solids_parameters.rigid_body_collision_parameters.use_push_out=true;
    solids_parameters.rigid_body_collision_parameters.use_analytic_collisions=false;
    //solids_parameters.analytic_test = true;
    solids_parameters.rigid_body_collision_parameters.contact_iterations=9;
    solids_parameters.rigid_body_collision_parameters.collision_iterations=9;
    solids_parameters.rigid_body_collision_parameters.use_shock_propagation=true;    
    solids_parameters.use_trapezoidal_rule_for_velocities=true;
}

	void init_heightmap(const char *sim, const char *render, const TV scale = TV(300, 300, 300), const TV offset = TV(0, -5, 0), const ROTATION<TV> rot = ROTATION<TV>(0, TV(0, 1, 0)),
		const OPENGL_MATERIAL& material = OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(1,1,1), 0.))) {

//		terrain = &tests.Add_Rigid_Body("terrain2_lowpoly", 1, (T) 1);

		if (sim[strlen(sim)-4] == '.')
			terrain = &tests.Add_Height_Field(sim, scale);
		else
			terrain = &tests.Add_Rigid_Body(sim, 1, (T)1);
		terrain->Rotation() = rot;
		terrain->X() = offset;
		terrain->is_static = true;
		terrain->Set_Name("terrain");
		terrain->Set_Coefficient_Of_Friction(tireFriction);
		terrain->Set_Coefficient_Of_Restitution(0);
		pushSimulationModel(terrain, material, material);
		if (strlen(render)) {
			terrain_render = driver->rigid_component->rigid_geometry_collection_rendering->Add_Height_Field(render, scale);
			terrain_render->Rotation() = rot;
			process_rendering(-1, material, material, offset);
		}
//		terrain->simplicial_object = 0;
//		pushRenderingModel(stream_type, "/sdcard/car/terrain2", material, material, -1, TV(0,-7,0), true);
	}

	void init_terrain(const char *sim, const char *render, const TV offset = TV(0,0,0), const T scale = (T) 1) {

		const OPENGL_MATERIAL& material = OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(1.0f, 1.0f, 1.0f), 1.));

		terrain = &tests.Add_Rigid_Body(sim, scale, (T) 1);
//		terrain = &tests.Add_Height_Field(sim, scale);
		terrain->X() = offset;
		terrain->is_static = true;
		terrain->Set_Name("ground");
		terrain->Set_Coefficient_Of_Friction(tireFriction);
		pushSimulationModel(terrain, material, material);
//		terrain_render = driver->rigid_component->rigid_geometry_collection_rendering->Add_Height_Field(render, scale);
//		process_rendering(-1, material, material, TV(0,-5,0));
//		terrain->simplicial_object = 0;
		pushRenderingModel(stream_type, render, material, material, -1, offset, true, scale);
	}

	void Scene0() {
		init_heightmap("C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/heightmap0_low.jpg", "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/heightmap0", TV(300,50,300));
		Add_Flag(TV(10, 3, 15));
		Add_Landmine(0, 0, 150, 150, 1000);
	}

	void Scene1() {
		ROTATION<TV> rot = ROTATION<TV>(pi / 2, TV(0, 1, 0));

		init_heightmap("level01", "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/level01_lopoly", TV(250, 60, 250), TV(72, 5, 90), rot);
	}

	void Add_Landmine(float centerx, float centerz, float dx, float dz, int number, float mine_chance=0.25, float excluded_radius=10.0) {
		need_explosions = true;

		for (int i = 0; i < number; ++i) {

			float x, y, z;
			do {
				x = centerx + dx * (rand() / (float) RAND_MAX - 0.5f);
				z = centerz + dz * (rand() / (float) RAND_MAX - 0.5f);
			} while (hypot(x - centerx, z - centerz) < excluded_radius);

			ANALYTIC_IMPLICIT_OBJECT<HEIGHT_FIELD<T> >* height_field = dynamic_cast<ANALYTIC_IMPLICIT_OBJECT<HEIGHT_FIELD<T> > * > (terrain->implicit_object->object_space_implicit_object);
			if (height_field) {
				y = -height_field->analytic.Signed_Distance(TV(x,0,z));
			} else {
				// TODO implement Metropolis-Hastings based terrain sampling based on density map (load from file)
				y = 0;
			}

			bool is_mine = (rand() / (float) RAND_MAX) < mine_chance;

			RIGID_BODY<TV>* rock = &tests.Add_Analytic_Box(TV(1,2,1));
			rock->is_static = true;
			rock->X() = TV(x, y + terrain->X()(2), z);
			pushSimulationModel(rock);
			if (is_mine) {
				rock->Set_Name("mine");
			}

			// Customize the rendering model.

			// (1) We have a variety of different rock models, so choose a random file.
			char filename[100];
			sprintf(filename, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/rock%d_lowpoly", (rand() % 3) + 1);

			// (2) Pick a random shade of gray for the rock color.
			int shade = rand() % 40 + 60;

			bool debug_mines = false;

			TV color = (debug_mines && is_mine) ? TV(1.0f, 0.0f, 0.0f) : TV((float)(shade / 255.), (float)(shade / 255.), (float)(shade / 255.));

			pushRenderingModel(
				stream_type,
				std::string(filename),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(color, 1.)),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(color, 1.)),
				-1,
				rock->X() + TV(0, 0.5, 0), // make sure the rocks don't get pushed underground
				false,
				0.5 // scale the model down to approximately match the simulation model size
			);
		}

		Setup_Explosions();
	}

	void Scene2() {
		init_heightmap("level02", "", TV(300, 300, 300), TV(0, 0, 0), ROTATION<TV>(pi, TV(0, 1, 0)),
			OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(0.8,0.525,0.172), 0.)));
		//init_terrain("level02tri", "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/level02tri");
		RIGID_BODY<TV>* funnel = &tests.Add_Rigid_Body("funnel", 2.1, (T)1);
		funnel->X() = TV(0, -10, 92);
		funnel->is_static = true;
		funnel->Set_Coefficient_Of_Restitution(0);
		funnel->Set_Mass(10);
		funnel->Update_Angular_Momentum();
		pushSimulationModel(funnel,
			OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(1.0, 1.0, 1.0), 1.)),
			OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(1.0, 1.0, 1.0), 1.))
			);

		button = &tests.Add_Rigid_Body("button", 2, (T)1);
		button->X() = TV(0, -13, 92);
		button->Set_Coefficient_Of_Restitution(1);
		button->Set_Mass(1);
		button->is_static = true;
		button->Update_Angular_Momentum();
		pushSimulationModel(button);

		ball = &tests.Add_Rigid_Body("sphere", 1.3, (T) 1);
		ball->X() = TV(0, 0, 75);
		ball->Set_Name("ball");
		ball->Set_Coefficient_Of_Restitution(1);
		ball->Set_Mass(10);
		ball->Update_Angular_Momentum();
		pushSimulationModel(ball);

		RIGID_BODY<TV>* w1 = &tests.Add_Rigid_Body("straightwall", 1, (T)1);
		w1->X() = TV(0, 0, -100);
		w1->Rotation() = ROTATION<TV>(pi / 2, TV(0, 1, 0));
		w1->Set_Coefficient_Of_Restitution(0.2);
		w1->is_static = true;
		w1->Set_Mass(10);
		w1->Update_Angular_Momentum();
		pushSimulationModel(w1);

		RIGID_BODY<TV>* w2 = &tests.Add_Rigid_Body("curvewall", 1, (T)1);
		w2->X() = TV(-7, 0, -97);
		w2->Rotation() = ROTATION<TV>(pi / 4, TV(0, 1, 0));
		w2->Set_Coefficient_Of_Restitution(0.2);
		w2->is_static = true;
		w2->Set_Mass(10);
		w2->Update_Angular_Momentum();
		pushSimulationModel(w2);

		RIGID_BODY<TV>* w6 = &tests.Add_Rigid_Body("straightwall", 1, (T)1);
		w6->X() = TV(-10, 0, -91);
		w6->Rotation() = ROTATION<TV>(pi , TV(0, 1, 0));
		w6->Set_Coefficient_Of_Restitution(0.2);
		w6->is_static = true;
		w6->Set_Mass(10);
		w6->Update_Angular_Momentum();
		pushSimulationModel(w6);

		RIGID_BODY<TV>* w3 = &tests.Add_Rigid_Body("curvewall", 1, (T)1);
		w3->X() = TV(-7, 0, -84);
		w3->Rotation() = ROTATION<TV>(pi * 3 / 4, TV(0, 1, 0));
		w3->Set_Coefficient_Of_Restitution(0.2);
		w3->is_static = true;
		w3->Set_Mass(10);
		w3->Update_Angular_Momentum();
		pushSimulationModel(w3);

		gate = &tests.Add_Rigid_Body("straightwall", 1, (T)1);
		gate->X() = TV(-1, 0, -81);
		gate->Rotation() = ROTATION<TV>(pi * 3 / 2, TV(0, 1, 0));
		gate->Set_Coefficient_Of_Restitution(0.2);
		gate->Set_Mass(10);
		gate->is_static = false;
		gate->Update_Angular_Momentum();
		pushSimulationModel(gate, OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(20./255., 151./255., 20./255.), 1.)),
			OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(20./255., 151./255., 20./255.), 1.)));

		gatest = new POINT_JOINT<TV>();
		gatest->Use_Phi_Constraint(0, 0);
		gatest->Use_Twist_Constraint(0., 0.);
		gatest->Use_Theta_Constraint(0., 0.);
		solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
				w3->particle_index, gate->particle_index,gatest);
		gatest->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., 4.3156),ROTATION<TV>(0., TV(0., 1., 0.))));
		gatest->Set_Joint_To_Parent_Frame(w3->Frame().Inverse_Times(
						gate->Frame()* FRAME<TV>(TV(0., 0., 4.3156),ROTATION<TV>(0., TV(0., 1., 0.)))));


		RIGID_BODY<TV>* w4 = &tests.Add_Rigid_Body("curvewall", 1, (T)1);
		w4->X() = TV(6, 0, -84);
		w4->Rotation() = ROTATION<TV>(pi * 5 / 4, TV(0, 1, 0));
		w4->Set_Coefficient_Of_Restitution(0.2);
		w4->is_static = true;
		w4->Set_Mass(10);
		w4->Update_Angular_Momentum();
		pushSimulationModel(w4);

		RIGID_BODY<TV>* w8 = &tests.Add_Rigid_Body("straightwall", 1, (T)1);
		w8->X() = TV(9, 0, -90);
		w8->Set_Coefficient_Of_Restitution(0.2);
		w8->is_static = true;
		w8->Set_Mass(10);
		w8->Update_Angular_Momentum();
		pushSimulationModel(w8);

		RIGID_BODY<TV>* w5 = &tests.Add_Rigid_Body("curvewall", 1, (T)1);
		w5->X() = TV(6, 0, -97);
		w5->Rotation() = ROTATION<TV>(-pi / 4, TV(0, 1, 0));
		w5->Set_Coefficient_Of_Restitution(0.2);
		w5->is_static = true;
		w5->Set_Mass(10);
		w5->Update_Angular_Momentum();
		pushSimulationModel(w5);

		Add_Flag(TV(0, 0, -90));
	}

	void Scene3() {
		float s = 2;
		init_terrain("level03_lowpoly", "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/level03_hipoly", TV(125 * s, -10 * s, 125 * s), s);
		terrain->Set_Coefficient_Of_Restitution(0.1);
		for (int i = 0; i < 50; ++i) {
			RIGID_BODY<TV>* rock = &tests.Add_Analytic_Box(TV(3,3,3));
			rock->X() = TV(125 * s + (-0.5+rand() / (float)RAND_MAX) * 30.0, 150 + i * 55, 125 * s + (-0.5+rand() / (float)RAND_MAX) * 30.0);
			rock->Set_Coefficient_Of_Restitution(0);
			rock->Set_Coefficient_Of_Friction(0.2);
			rock->Set_Mass(100);
			rocks.Append(rock);
			pushSimulationModel(rock);
			char filename[100];
			sprintf(filename, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/rock%d_lowpoly", (rand() % 3) + 1);

			// (2) Pick a random shade of gray for the rock color.
			int shade = rand() % 40 + 60;

			TV color = TV(shade / 255., shade / 255., shade / 255.);
			pushRenderingModel(
				stream_type,
				std::string(filename),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(color, 1.)),
				OPENGL_MATERIAL::Matte(OPENGL_COLOR(color, 1.)),
				-1,
				rock->X() + TV(0, 0.5, 0), // make sure the rocks don't get pushed underground
				false,
				2 // scale the model down to approximately match the simulation model size
			);
		}
		Add_Flag(TV(125 * s, 100 * s, 125 * s));
	}

	void Add_Bridge(BRIDGE_TYPE type, int num, T scale = (T)1, T mass = (T)10, const FRAME<TV>& frame = FRAME<TV>(TV(0, 0, 0), ROTATION<TV>(0, TV(0, 1, 0))), bool is_static = false) {
		RIGID_BODY<TV>** planks =  (RIGID_BODY<TV>**)malloc(sizeof(RIGID_BODY<TV>*) * num);

		const T angular_damping = 100;

		switch (type) {
		case PLANKS:
			{
				for (int i = 0; i < num / 2; ++i) {
					planks[i] = &tests.Add_Rigid_Body("bridge_plank", 4 * scale, (T)1);
					planks[i]->Set_Coefficient_Of_Restitution(0.1f);
					planks[i]->X() = frame.r.Rotation_Matrix() * TV(0, 0, (is_static?10:5) * i * scale) + frame.t;
					planks[i]->Rotation() = frame.r;
					planks[i]->Set_Mass(mass);
					planks[i]->is_static = is_static;
					pushSimulationModel(planks[i], OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)),
						OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)));
					if (i != 0 && !is_static) {
						for (int j = 0; j < 2; ++j) {
							POINT_JOINT<TV>* st = new POINT_JOINT<TV>();

							solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
								planks[i - 1]->particle_index, planks[i]->particle_index,st);
							st->Set_Joint_To_Child_Frame(FRAME<TV>(TV((j-0.5)*10, 0., -5 * scale),ROTATION<TV>(0., TV(0., 0, 0.))));
							st->Set_Joint_To_Parent_Frame(FRAME<TV>(TV((j-0.5)*10, 0., 5 * scale),ROTATION<TV>(0., TV(0., 0, 0.))));
						}
					}
				}
				for (int i = num - 1; i >= num / 2; --i) {
					planks[i] = &tests.Add_Rigid_Body("bridge_plank", 4 * scale, (T)1);
					planks[i]->Set_Coefficient_Of_Restitution(0.1f);
					planks[i]->X() = frame.r.Rotation_Matrix() * TV(0, 0, (float)(((is_static ? 0 : 5) * (num - 1) - 0.1 * num + (is_static ? 10 : 5) * i) * scale)) + frame.t;
					planks[i]->Rotation() = frame.r;
					planks[i]->Set_Mass(mass);
					planks[i]->is_static = is_static;
					pushSimulationModel(planks[i], OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)),
						OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)));
					if (i != num - 1  && !is_static) {
						for (int j = 0; j < 2; ++j) {
							POINT_JOINT<TV>* st = new POINT_JOINT<TV>();

							solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
								planks[i + 1]->particle_index, planks[i]->particle_index,st);
							st->Set_Joint_To_Child_Frame(FRAME<TV>(TV((j-0.5)*10, 0., 5 * scale),ROTATION<TV>(0., TV(0., 0, 0.))));
							st->Set_Joint_To_Parent_Frame(FRAME<TV>(TV((j-0.5)*10, 0., -5 * scale),ROTATION<TV>(0., TV(0., 0, 0.))));
						}
					}
				}
				if (!is_static) {
					for (int j = 0; j < 2; ++j) {
						POINT_JOINT<TV>* st = new POINT_JOINT<TV>();
						st->Use_Twist_Constraint((float)-pi, (float)pi);
						st->Use_Phi_Constraint(0., 0.);
						st->Use_Theta_Constraint(0, 0);

						solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
							planks[num / 2]->particle_index, planks[num / 2 - 1]->particle_index,st);
						st->Set_Joint_To_Child_Frame(FRAME<TV>(TV((j-0.5f)*10, 0., 5 * scale),ROTATION<TV>(0., TV(0., 0, 0.))));
						st->Set_Joint_To_Parent_Frame(FRAME<TV>(TV((j-0.5f)*10, 0., -5 * scale),ROTATION<TV>(0., TV(0., 0, 0.))));
						st->angular_damping = angular_damping;
					}
				}
				planks[num - 1]->is_static = true;
				planks[0]->is_static = true;
			}
			break;

		case RUBBER:
			{
				for (int i = 0; i < num; ++i) {
					planks[i] = &tests.Add_Rigid_Body("rubber_plank", 4 * scale, (T)1);
					planks[i]->X() = frame.r.Rotation_Matrix() * TV(0, 0, 10 * scale * i) + frame.t;
					planks[i]->Rotation() = frame.r;
					pushSimulationModel(planks[i], OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)),
						OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)));
					if (i != 0 ) {
						POINT_JOINT<TV>* st = new POINT_JOINT<TV>();
						st->Use_Twist_Constraint((float)(-pi / 12), (float)(pi / 12));
						st->Use_Phi_Constraint(0., 0.);
						st->Use_Theta_Constraint(0, 0);

						solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
							planks[0]->particle_index, planks[i]->particle_index,st);
						st->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., 0),ROTATION<TV>(0., TV(0., 0, 0.))));
						st->Set_Joint_To_Parent_Frame(planks[0]->Frame().Inverse_Times(
									planks[i]->Frame()* FRAME<TV>(TV(0., 0., 0),ROTATION<TV>(0., TV(0., 0., 0.)))));
					}
				}
				planks[0]->is_static = true;
			}
			break;

		case SPRING:
			{
				RIGID_BODY<TV>* ground = &tests.Add_Ground(1.0, -60, 0, 1000);
				pushSimulationModel(ground, OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)),
					OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)));
				for (int i = 0; i < num; ++i) {
					planks[i] = &tests.Add_Rigid_Body("spring_plank", 4, (T)1);
					planks[i]->X() = frame.r.Rotation_Matrix() * TV(0, 0, scale * 10 * i) + frame.t;
					planks[i]->Rotation() = frame.r;
					pushSimulationModel(planks[i], OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)),
						OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)));
					PRISMATIC_POINT_JOINT<TV>* sj = new PRISMATIC_POINT_JOINT<TV>();
					//sj->Use_Phi_Constraint(-pi/24.,pi/24.);
					sj->Use_Phi_Constraint(0., 0.);
					sj->Use_Theta_Constraint(0., 0.);
					sj->Use_Twist_Constraint(0., 0.);
				    sj->constrain.Fill(true);
				    sj->prismatic_min=TV();
				    sj->prismatic_max=TV();
				    sj->prismatic_min(2)=-10000;
				    sj->prismatic_max(2)=10000;
					solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(
							planks[i]->particle_index, ground->particle_index,sj);
					sj->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0., 0., 0.), ROTATION<TV>(0., TV(0., 1., 0.))));
					sj->Set_Joint_To_Parent_Frame(
							planks[i]->Frame().Inverse_Times(ground->Frame()	* FRAME<TV>(TV(0., 0., 0),	ROTATION<TV>(0., TV(0., 1., 0.)))));
					sj->Add_Spring(planks[i]->Frame().Inverse_Times(ground->Frame()*TV(0.,0.,0)), TV(0.,0.0,0),500,10,0.0,10.);
				}
			}
			break;

		default:
			PHYSBAM_FATAL_ERROR("unknown bridge type");
		}
		//free(planks);
	}

	void Scene4() {
		init_terrain("level05", "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/level05");

		Add_Landmine(-85, 170, 75, 90, 500, 0.25, 0);

		Add_Bridge(PLANKS, 9, 1, 30, FRAME<TV>(TV(60, -2, 130), ROTATION<TV>(0, TV(0,1,0))) );

		Add_Flag(TV(0, 0, 250));
	}
	
	void Scene5() {
		init_heightmap("C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/desert_heightmap_lopoly.jpg", "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/desert_heightmap", TV(300,15,300));
		Add_Flag(TV(10, 3, 15));

		return;
		RIGID_BODY<TV>*ground = &tests.Add_Ground(1.0, -20, 0, 1000);
		RIGID_BODY<TV>**planks = (RIGID_BODY<TV>**)malloc(sizeof(RIGID_BODY<TV>*) * 10);
		pushSimulationModel(ground, OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)), OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)));
		for (int i = 0; i < 10; ++i) {
			planks[i] = &tests.Add_Rigid_Body("spring_plank", 4, (T)1);
			planks[i]->X() = TV(0, -10, 10 * i);
			pushSimulationModel(planks[i], OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)), OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)));
			PRISMATIC_POINT_JOINT<TV>* sj = new PRISMATIC_POINT_JOINT<TV>();
			sj->Use_Phi_Constraint(0,0);
			sj->Use_Theta_Constraint(0,0);
			sj->Use_Twist_Constraint(0,0);
			sj->prismatic_min = TV();
			sj->prismatic_max = TV();
			sj->prismatic_min(2) = -100;
			sj->prismatic_max(2) = 100;
			solid_body_collection.rigid_body_collection.articulated_rigid_body.joint_mesh.Add_Articulation(planks[i]->particle_index, ground->particle_index, sj);
			sj->Set_Joint_To_Child_Frame(FRAME<TV>(TV(0,0,0),ROTATION<TV>(0,TV(0,0,0))));
			sj->Set_Joint_To_Parent_Frame(
				planks[i]->Frame().Inverse_Times(ground->Frame() * FRAME<TV>(TV(0,0,0), ROTATION<TV>(0, TV(0,1,0)))));
			sj->Add_Spring(planks[i]->Frame().Inverse_Times(ground->Frame() * TV(0, 0, 0)), TV(0,0,0), 500, 40, 0, 10 / 3);
		}
		//free(planks);
	}

	void Scene6() {
		init_terrain("mountain_ice_lowpoly", "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/mountain_ice", TV(0, -12, -300), 10);

		bool close_flag_demo = true;
		if (close_flag_demo) {
			Add_Flag(TV(10, -3, 0));
		} else { 
			Add_Flag(TV(125, 100, 125));
		}

		ground = &tests.Add_Ground(1.0,-3,0,0.001);
		ground->is_static = true;
		ground->Set_Name("ice");
		//ground->X()=TV(0, -10, 0)
		pushSimulationModel(ground, OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 10./255.), 1.)),
			 OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 10./255.), 1.)));
	}

	void Scene7() {
		/*
		 * Scene 7: bridge chain.
		 * This level contains a number of platforms, connected by bridges.
		 * The user starts at the first platform, and the flag is on the last platform.
		 */

		// Set up parameters for the platform positions.
		const size_t num_platforms = 3;
		TV platforms[num_platforms] = {
		    TV(0, 0, 0), TV(0, 0, 100), TV(150, -10, 200)
		};

		const OPENGL_MATERIAL& platform_material = OPENGL_MATERIAL::Matte(OPENGL_COLOR(TV(0.8, 0.8, 0.8), 1));

		// Define physical properties for the platforms and bridge.
		// You may wish to modify these values.
		const T platform_size = 25; // the diameter of the platforms
		const T platform_height = 1; // the height of the platforms on the Y axis
		const T bridge_plank_depth = 8.04; // the depth (Z-axis) of a bridge plank
		const T bridge_plank_spacing = 2; // the spacing between consecutive bridge planks
		const int resolution_radius = 32; // the vertex resolution on the radial axis
		const T platform_friction = 0.8; // the coefficient of friction for the platforms

		// By default, the platform cylinders are Z-facing. This rotation fixes that.
		const ROTATION<TV> platform_rotation = ROTATION<TV>(pi/2, TV(1, 0, 0));

		// Offset the platforms down so a platform at (0, 0, 0) has its top face at y = 0.
		const TV platform_offset = TV(0, -platform_height / (T) 2, 0);

		// Create the first platform.
		// This is outside the loop because there is one fewer bridge than there are platforms.
		// (This is a fencepost problem.)
		RIGID_BODY<TV>& first_platform = tests.Add_Analytic_Cylinder(platform_height, platform_size / (T) 2, resolution_radius);
		first_platform.X() = platforms[0];
		first_platform.Rotation() = platform_rotation;
		first_platform.Set_Coefficient_Of_Friction(platform_friction);
		first_platform.is_static = true;
		pushSimulationModel(&first_platform, platform_material, platform_material);

		/*
		 * Loop for each pair:
		 *
		 *   (a, b) = (platforms[0], platforms[1]), (platforms[1], platforms[2]), ...
		 *
		 * At each pair:
		 *   - create the bridge from 'a' to 'b'
		 *   - create the platform at 'b'
		 */
		for (size_t i = 1; i < num_platforms; ++i) {
			// Get the endpoints from the list.
			const TV a = platforms[i - 1];
			const TV b = platforms[i];

			// The vector from the center of the platform to the edge of the
			// platform, in the direction of the bridge, projected onto the
			// plane of the platform.
			const TV platform_r = (b - a).Projected_Orthogonal_To_Unit_Direction(TV(0, 1, 0)).Normalized() * platform_size / 2;

			// The unit vector in the direction that the bridge will span.
			TV bridge_dir = ((b - a) - platform_r * 2).Normalized();

			// The center point of the first plank.
			const TV bridge_start = a + platform_r + bridge_dir * (bridge_plank_depth + bridge_plank_spacing) / 2;

			// By default, bridges face in the direction (0, 0, 1).
			// We need to rotate the bridge to face in the correct direction.
			//
			// We use (0,001, 0, 0.999) instead of (0, 0, 1) so that bridges
			// facing the (0, 0, -1) direction don't have a zero cross product.
			const T angle1 = TV::Angle_Between(TV(0.001, 0, 0.999), bridge_dir);
			const TV roll1 = TV::Cross_Product(TV(0.001, 0, 0.999), bridge_dir);
			const ROTATION<TV> r1 = ROTATION<TV>(angle1, roll1);

			// Our first rotation will cause the bridge to bank, because the
			// vector will roll as it is rotated.
			//
			// Let the "normal" of a bridge be the vector perpendicular to the
			// y-axis and to the direction the bridge is facing. For example,
			// the default bridge normal is (1, 0, 0). The following vector is
			// the desired bridge normal for this bridge.
			const TV desired_normal = TV::Cross_Product(TV(0, 1, 0),bridge_dir);

			// To correct the banking, we calculate the rotation between the
			// actual normal and the desired normal.
			const TV actual_normal = r1.Rotation_Matrix() * TV(0.999, 0, 0.001);
			const TV roll2 = TV::Cross_Product(actual_normal, desired_normal);
			const T angle2 = TV::Angle_Between(actual_normal, desired_normal);
			const ROTATION<TV> r2 = ROTATION<TV>(angle2, roll2);

			// Finally, we combine the initial rotation with the bank
			// correction to get the final rotation for the bridge.
			const ROTATION<TV> rotation = r2 * r1;

			// Calculate the number of planks needed. Round down.
			const int plank_count = (int) (
				((b - a) - platform_r * 2).Magnitude() / (bridge_plank_depth + bridge_plank_spacing)
			);

			// In general, the integer number of planks won't exactly span the
			// gap, because we're rounding down. Calculate the error incurred
			// and move this platform and all subsequent platforms to correct.

			// The ideal_edge is the where edge of the last platform would be
			// if there were no error.
			const TV ideal_edge = a + platform_r + bridge_dir * (plank_count * (bridge_plank_depth + bridge_plank_spacing) + bridge_plank_spacing);

			// The current_edge is where the edge actually is.
			const TV current_edge = b - platform_r;

			// Calculate the error and move the platforms.
			const TV error = ideal_edge - current_edge;
			for (size_t j = i; j < num_platforms; ++j) {
				platforms[j] += error;
			}
			const TV new_b = b + error;

			// Create the platform.
			RIGID_BODY<TV>& platform = tests.Add_Analytic_Cylinder(platform_height, platform_size / (T) 2, resolution_radius);
			platform.X() = new_b + platform_offset;
			platform.Rotation() = platform_rotation;
			platform.Set_Coefficient_Of_Friction(platform_friction);
			platform.is_static = true;
			pushSimulationModel(&platform, platform_material, platform_material);

			// Create the bridge.
			if (i == 1)
				Add_Bridge(RUBBER, plank_count, 1, 30, FRAME<TV>(bridge_start, rotation));
			else
				Add_Bridge(PLANKS, plank_count, 1, 30, FRAME<TV>(bridge_start, rotation));
		}

		// Add a flag at the last platform.
		Add_Flag(platforms[num_platforms - 1]);
	}

	void Add_Flag(TV where = TV(0, 0, 0), T flagScale=2.0, T flagElevation=6.0) {
		// the flag is 6 wide by 4 high

		T flagpoleHeight = 4 * flagScale + flagElevation;		
		RIGID_BODY<TV>* flagpole = &tests.Add_Analytic_Cylinder((T) flagpoleHeight, (T) 0.1);
		flagpole->Set_Name("flagpole");
		flagpole->X() = where + TV(0, flagpoleHeight / 2, 0);
		flagpole->Rotation() = ROTATION<TV>((float)(pi / 2), TV(1.0f, 0.0f, 0.0f));
		flagpole->is_static = true;
		pushSimulationModel(flagpole, OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1), 1.)),
			OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1), 1.)));
		RIGID_GEOMETRY<TV>* pole_render = driver->rigid_component->rigid_geometry_collection_rendering->Add_Analytic_Cylinder((T) flagpoleHeight, (T) 0.1);
		pole_render->X() = where + TV(0, flagpoleHeight / 2, 0);
		pole_render->Rotation() = ROTATION<TV>((float)(pi / 2), TV(1.0f,0.0f,0.0f));
		process_rendering(-1, OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1), 1.)),
			OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1), 1.)));

		flag = pushRenderingModel(stream_type, "C:/Army2015/PhysDrive-PC/Public_Data/Archives/Rigid_Bodies/flag",
    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)),
    		OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(1,1,1),1)),
    		-1, TV(0, 2.98f, 0), false, 2.0f, true, false, false, false, false);
		flag->omit_simulation_binding = true;
		flag->X() = TV(0, flagElevation + 2 * flagScale, 0) + where;
		TV y_axis = TV(0, 1, 0);
		TV z_axis = TV(0, 0, 1);
		TV pole_axis = jeep->Frame().r.Rotation_Matrix() * y_axis;
		TV normal = TV::Cross_Product(pole_axis, wind_direction);
		TV roll1 = TV::Cross_Product(y_axis, pole_axis);
		T angle1 = TV::Angle_Between(y_axis, pole_axis);
		ROTATION<TV> r1 = ROTATION<TV>(angle1, roll1);
		z_axis = r1.Rotation_Matrix() * z_axis;
		TV roll2 = TV::Cross_Product(z_axis, normal);
		T angle2 = TV::Angle_Between(z_axis, normal);
		ROTATION<TV> r2 = ROTATION<TV>(angle2, roll2);
		flag->Rotation() = r2 * r1;

	    flag->simplicial_object->textures = car_flag->simplicial_object->textures;
	}


	void Add_Fog(){
		GLfloat density = 0.03;
		GLfloat fogColor[4] = {0.7, 0.7, 0.75, 1.0};


		glEnable(GL_FOG);
		glFogi (GL_FOG_MODE, GL_EXP2);
		glFogfv (GL_FOG_COLOR, fogColor);
		glFogf (GL_FOG_DENSITY, density);
		glHint(GL_FOG_HINT, GL_NICEST);
	}
	void pushSimulationModel(RIGID_BODY<TV>* simulation_obj,
		const OPENGL_MATERIAL& front = OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 20./255.), 1.)),
		const OPENGL_MATERIAL& back =  OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 20./255.), 1.))
	) {
		driver->rigid_component->num_basic_object++;
		driver->rigid_component->next_for_rendering.Append(0);
		driver->rigid_component->front_material_collection.Append(front);
		driver->rigid_component->back_material_collection.Append(back);
		firstRenderModel = true;
	}

	void process_rendering(int fid = -1,
		const OPENGL_MATERIAL& front = OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 20./255.), 1.)),
		const OPENGL_MATERIAL& back =  OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 20./255.), 1.)),
		const TV offset = TV(0, 0, 0)
	) {
		if (fid == -1)
			fid = driver->rigid_component->next_for_rendering.Size();
		driver->rigid_component->parent_index.Append(fid);
		driver->rigid_component->next_for_rendering(driver->rigid_component->next_for_rendering.Size())++;
		int rid = driver->rigid_component->rigid_geometry_collection_rendering->particles.array_collection->Size();
		FRAME<TV> F_ai1(driver->rigid_component->rigid_geometry_collection_simulation->particles.X(fid),
			driver->rigid_component->rigid_geometry_collection_simulation->particles.rotation(fid));
		FRAME<TV> F_ri(driver->rigid_component->rigid_geometry_collection_rendering->particles.X(rid)+offset,
			driver->rigid_component->rigid_geometry_collection_rendering->particles.rotation(rid));
		driver->rigid_component->relative_frame_rendering.Append(F_ai1.Inverse_Times(F_ri));
		if (!firstRenderModel) {
			driver->rigid_component->num_basic_object++;
			driver->rigid_component->front_material_collection.Append(front);
			driver->rigid_component->back_material_collection.Append(back);
		} else {
			firstRenderModel = false;
			driver->rigid_component->front_material_collection(driver->rigid_component->num_basic_object) = front;
			driver->rigid_component->back_material_collection(driver->rigid_component->num_basic_object) = back;
		}		
	}

	RIGID_GEOMETRY<TV>*  pushRenderingModel(const STREAM_TYPE stream_type, const std::string& basename, 
		const OPENGL_MATERIAL& front = OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 20./255.), 1.)),
		const OPENGL_MATERIAL& back =  OPENGL_MATERIAL::Metal(OPENGL_COLOR(TV(151./255., 10./255., 20./255.), 1.)),
		int fid = -1,
		const TV offset = TV(0, 2.98, 0),
		const bool use_ccw = false,
		const T scaling_factor_rendering = 1.0f,
		const bool read_simplicial_boundary = true, const bool read_implicit_object = false,
        const bool read_simplicial_interior = false, const bool read_rgd_file = false,
        const bool read_tex_file = true
    ) {
		int t_id = driver->rigid_component->rigid_geometry_collection_rendering->Add_Rigid_Geometry(
			stream_type, basename, scaling_factor_rendering, 
			read_simplicial_boundary, read_implicit_object, read_simplicial_interior, read_rgd_file, read_tex_file);
		RIGID_GEOMETRY<TV>* result = driver->rigid_component->rigid_geometry_collection_rendering->particles.rigid_geometry(t_id);
		int rid = driver->rigid_component->rigid_geometry_collection_rendering->particles.array_collection->Size();
		driver->rigid_component->rigid_geometry_collection_rendering->particles.rigid_geometry(rid)->name = basename;
//		driver->rigid_component->use_rendering_ccws.Append(use_ccw);
		process_rendering(fid, front, back, offset);
//		driver->rigid_component->rigid_geometry_collection_rendering->Add_Rigid_Geometry(
//			stream_type, basename, scaling_factor_rendering, 
//			read_simplicial_boundary, read_implicit_object, read_simplicial_interior, read_rgd_file);
		return result;
    }


void Add_Body(RIGID_BODY<TV>* & target, const TV pos, const std::string& filename, const T scale)
{
	target=&tests.Add_Rigid_Body(filename, scale, (T).5);
	target->Rotation()=ROTATION<TV>(2*acos(1),TV(0.,1.,0.));
	target->X()=pos;
        target->Set_Coefficient_Of_Restitution(0.5);
        target->Set_Name("target");
        target->Set_Mass(1);
      	target->Update_Angular_Momentum();

}



void Load_Targerts(const std::string& filename)
{
	std::ifstream fin;
	fin.open(filename.c_str());
	if(!fin)
	{
		std::cerr<<"error: unable to open input file."<<std::endl;
		abort();
	}

	T x, y, z;
	fin>>target_cnt;	
	target_list = new RIGID_BODY<TV>* [target_cnt];
	cur_target_id = 0;
	for(unsigned int i=0; i<target_cnt; i++)
	{
		fin>>x>>y>>z;
		Add_Body(target_list[i], TV(x, y, z), "box", .5);   	
	}
}

void Load_Obstacles(const std::string& filename)
{
	std::ifstream fin;
	fin.open(filename.c_str());
	if(!fin)
	{
		std::cerr<<"error: unable to open input file."<<std::endl;
		abort();
	}

	T x, y, z;
	T scale, mass;
	std::string name; 
	fin>>obst_cnt;	
	obst_list = new RIGID_BODY<TV>* [obst_cnt];
	for(unsigned int i=0; i<obst_cnt; i++)
	{
		fin>>x>>y>>z>>scale>>mass>>name;
		RIGID_BODY<TV>* obst = obst_list[i];

		obst=&tests.Add_Rigid_Body(name, scale, (T)1);
		obst->Rotation()=ROTATION<TV>(2*acos(1),TV(0.,1.,0.));
		obst->X()=TV(x, y, z);
        	obst->Set_Coefficient_Of_Restitution(0.5);
        	obst->Set_Name("obst");
        	obst->Set_Mass(mass);
    		//obst->is_static=true;
      		obst->Update_Angular_Momentum();
  	
	}
}


public:
	bool forward;
	bool back;
	T speed_limit;
	int dst_wise;
	T wheels_angle;
	T dst_angle;
	bool nitro;
	int trig_nitro;
	bool reverse;
	bool reverse_ref;
	float curr_speed;
	int curr_accel;
	bool lift;
    RIGID_BODY<TV> *jeep;

public:
	//class variables for jeep sim
	//int jeep_particle_index;
	/*
	static const T y_offset=5;
	static const T tire_offset = -0.;
	static const T sphere1_offset = -0.7;
	static const T sphere2_offset = -0.5;
	static const T sphere3_offset = -0.9;
	static const T deltat = 0.04166;
*/
	ARTICULATED_RIGID_BODY<TV>* arb;

	RIGID_GEOMETRY<TV> *car_flag;
	RIGID_GEOMETRY<TV> *car_pole;
	RIGID_GEOMETRY<TV> **explosions;
    //Rigid Bodies:
    RIGID_BODY<TV> *jeepbody;

    RIGID_BODY<TV> *frontlefttire;
    RIGID_BODY<TV> *frontrighttire;
    RIGID_BODY<TV> *backlefttire;
    RIGID_BODY<TV> *backrighttire;
    RIGID_BODY<TV> *button;

    RIGID_BODY<TV> *flsp;
    RIGID_BODY<TV> *frsp;
    RIGID_BODY<TV> *blsp;
    RIGID_BODY<TV> *brsp;
    
	RIGID_BODY<TV> *backleftsphere;
    RIGID_BODY<TV> *backrightsphere;
    RIGID_BODY<TV> *frontleftsphere;
    RIGID_BODY<TV> *frontrightsphere;
	
    RIGID_BODY<TV> *terrain;
    RIGID_BODY<TV> *ball;
    RIGID_BODY<TV> *gate;
    RIGID_BODY<TV> *ground;

    RIGID_GEOMETRY<TV> *terrain_render;
	//Joints (with spheres):
	POINT_JOINT<TV>* j_jb_fls;
	POINT_JOINT<TV>* j_jb_frs;
	POINT_JOINT<TV>* j_jb_bls;
	POINT_JOINT<TV>* j_jb_brs;
	POINT_JOINT<TV>* j_fls_flt;
	POINT_JOINT<TV>* j_frs_frt;
	POINT_JOINT<TV>* j_bls_blt;
	POINT_JOINT<TV>* j_brs_brt;
	POINT_JOINT<TV>* gatest;

	RIGID_GEOMETRY<TV>* flag;
   //Joints:
   PRISMATIC_POINT_JOINT<TV> *backleftsj;
   POINT_JOINT<TV> *backleftst;
   PRISMATIC_POINT_JOINT<TV> *backrightsj;
   POINT_JOINT<TV> *backrightst;
   PRISMATIC_POINT_JOINT<TV> *frontleftsj;
   POINT_JOINT<TV> *frontleftst;
   PRISMATIC_POINT_JOINT<TV> *frontrightsj;
   POINT_JOINT<TV> *frontrightst;


   //Targets and Obstacles:
    ARRAY<RIGID_BODY<TV>* > rocks;
	RIGID_BODY<TV>* target;
	RIGID_BODY<TV>** target_list;
	unsigned int target_cnt;
	unsigned int cur_target_id;
	RIGID_BODY<TV>** obst_list;
	unsigned int obst_cnt;
};
}
#endif
