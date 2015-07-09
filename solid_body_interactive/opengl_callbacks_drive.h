//#####################################################################

//#####################################################################
// Class OPENGL_BASIC_CALLBACKS
//#####################################################################
#ifndef __OPENGL_CALLBACKS_DRIVE__
#define __OPENGL_CALLBACKS_DRIVE__

#include <PhysBAM_Tools/Log/LOG.h>
#include <PhysBAM_Tools/Read_Write/Vectors/READ_WRITE_VECTOR.h>

#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_CALLBACK.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_WORLD.h>
#include <PhysBAM_Geometry/Solids_Geometry/RIGID_GEOMETRY_COLLECTION.h>
namespace PhysBAM{

//#####################################################################
// Callback for setting gas on, brake off 
//#####################################################################
//
class OPENGL_CALLBACK_FORWARD:public OPENGL_CALLBACK
{
private: 
	bool *forward;
	bool *back;
	int *curr_accel;
public:
    OPENGL_CALLBACK_FORWARD(bool *forward_input, bool *back_input,int *curr_accel_input)
        :forward(forward_input), back(back_input),curr_accel(curr_accel_input)
    {}

    void operator()()
    {
    	*forward = true;
		*back = false;
	if (*curr_accel >= 20){
		*curr_accel = 20;
	} else *curr_accel += 1;
    }

    void Print(std::ostream& out)
    {
        out<<"Toggle Accelerater";
    }
};
//#####################################################################
// Callback for setting brake on, gas off
//#####################################################################
//
class OPENGL_CALLBACK_BACK:public OPENGL_CALLBACK
{
private:
	bool *forward;
	bool *back;
	int *curr_accel;
public:
    OPENGL_CALLBACK_BACK(bool *forward_input, bool *back_input, int *curr_accel_input)
        :forward(forward_input), back(back_input), curr_accel(curr_accel_input)
    {}

    void operator()()
    {
    	*forward = false;
		*back = true;
        if (*curr_accel == 0){
            *curr_accel = 0;
        } else {
            *curr_accel -= 1;
        }
    }

    void Print(std::ostream& out)
    {
        out<<"Toggle Break";
    }
};
//#####################################################################
// Class 
//#####################################################################
//
class OPENGL_CALLBACK_TURNLEFT:public OPENGL_CALLBACK
{
private:
	int *wise;
public:
    OPENGL_CALLBACK_TURNLEFT(int *wise_input)
        :wise(wise_input)
	{}

    void operator()()
    {
    	*wise = 1;
    }

    void Print(std::ostream& out)
    {
        out<<"Turn Left";
    }
};
//#####################################################################
// Callback to set the car to be turning right 
//#####################################################################
//
class OPENGL_CALLBACK_TURNRIGHT:public OPENGL_CALLBACK
{
private:
	int *wise;
public:
    OPENGL_CALLBACK_TURNRIGHT(int *wise_input)
        :wise(wise_input)
	{}

    void operator()()
    {
    	*wise = -1;
    }

    void Print(std::ostream& out)
    {
        out<<"Turn Right";
    }
};
//#####################################################################
// Callback to activate nitro
//#####################################################################
//
class OPENGL_CALLBACK_NITRO:public OPENGL_CALLBACK
{
private:
	bool *nitro;
public:
    OPENGL_CALLBACK_NITRO(bool *nitro_input)
        :nitro(nitro_input)
	{}

    void operator()()
    {
    	*nitro = true;
    }

    void Print(std::ostream& out)
    {
        out<<"Turn Right";
    }
};
//#####################################################################
// Callback to toggle reverse/forward
//#####################################################################
//
class OPENGL_CALLBACK_REVERSE:public OPENGL_CALLBACK
{
private:
	bool *reverse;
	bool *forward;
	bool *reverse_ref;
	int *curr_accel;
public:
    OPENGL_CALLBACK_REVERSE(bool *reverse_input, bool *forward_input, bool *reverse_ref_input, int *curr_accel_input)
        :reverse(reverse_input), forward(forward_input), reverse_ref(reverse_ref_input),curr_accel(curr_accel_input)
	{}

    void operator()()
    {
    	*reverse = !(*reverse);
        *reverse_ref = true;
        *forward = false;
        *curr_accel = 0;
    }

    void Print(std::ostream& out)
    {
        out<<"Turn Right";
    }
};
//#####################################################################
// Class 
//#####################################################################
//
class OPENGL_CALLBACK_TOGGLE_JEEP:public OPENGL_CALLBACK
{
private:
	typedef VECTOR<float,3> TV;
	int *jeep_display_mode;
	RIGID_GEOMETRY_COLLECTION<TV>* rigid_geometry_collection;
public:
    OPENGL_CALLBACK_TOGGLE_JEEP(int *jeep_display_mode_input, RIGID_GEOMETRY_COLLECTION<TV>* rigid_geometry_collection_input)
        :jeep_display_mode(jeep_display_mode_input), rigid_geometry_collection(rigid_geometry_collection_input)
	{}

    void operator()()
    {
		if(*jeep_display_mode < 2) {
            (*jeep_display_mode) = (*jeep_display_mode)+1;
        } else {
            *jeep_display_mode = 0;
        }
		
		//delete &(rigid_geometry_collection->particles);
        //delete rigid_geometry_collection;
		//rigid_geometry_collection=new RIGID_GEOMETRY_COLLECTION<TV>(*new RIGID_GEOMETRY_PARTICLES<TV>(),0);
	}

    void Print(std::ostream& out)
    {
        out<<"Toggle jeep";
    }
};
//#####################################################################
//#####################################################################
// Class 
//#####################################################################
//
class OPENGL_CALLBACK_LIFT:public OPENGL_CALLBACK
{
private:
	bool *lift;
public:
    OPENGL_CALLBACK_LIFT(bool *lift_input)
        :lift(lift_input)
	{}

    void operator()()
    {
     	*lift = !(*lift);
    }

    void Print(std::ostream& out)
    {
        out<<"Turn Right";
    }
};
//#####################################################################
}
#endif

