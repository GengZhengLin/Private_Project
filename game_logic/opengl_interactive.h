#ifndef __INTERACTIVE_VISUALIZATION__
#define __INTERACTIVE_VISUALIZATION__
#include <PhysBAM_Tools/Grids_RLE/RLE_GRID_ITERATOR_CELL_3D.h>
#include <PhysBAM_Tools/Grids_Uniform/GRID.h>
#include <PhysBAM_Tools/Parsing/PARAMETER_LIST.h>
#include <PhysBAM_Tools/Parsing/PARSE_ARGS.h>
#include <PhysBAM_Tools/Read_Write/Arrays/READ_WRITE_ARRAY.h>
#include <PhysBAM_Tools/Read_Write/Grids_Uniform/READ_WRITE_GRID.h>
#include <PhysBAM_Tools/Read_Write/Grids_Uniform_Arrays/READ_WRITE_ARRAYS.h>
#include <PhysBAM_Tools/Read_Write/Grids_Uniform_Arrays/READ_WRITE_FACE_ARRAYS.h>
#include <PhysBAM_Tools/Read_Write/Utilities/FILE_UTILITIES.h>
#include <PhysBAM_Tools/Utilities/PROCESS_UTILITIES.h>
#include <PhysBAM_Tools/Grids_Uniform_Arrays/FACE_ARRAYS.h>
#include <PhysBAM_Tools/Log/LOG.h>
#include <PhysBAM_Tools/Point_Clouds/POINT_CLOUD_SUBSET.h>
#include <PhysBAM_Tools/Read_Write/Data_Structures/READ_WRITE_PAIR.h>
#include <PhysBAM_Geometry/Geometry_Particles/GEOMETRY_PARTICLES_FORWARD.h>
#include <PhysBAM_Geometry/Basic_Geometry/RAY.h>
#include <PhysBAM_Geometry/Basic_Geometry/SPHERE.h>
#include <PhysBAM_Geometry/Basic_Geometry_Intersections/RAY_PLANE_INTERSECTION.h>
#include <PhysBAM_Geometry/Read_Write/Geometry/READ_WRITE_TRIANGULATED_SURFACE.h>
#include <PhysBAM_Geometry/Topology_Based_Geometry/TRIANGULATED_SURFACE.h>
#include <PhysBAM_Geometry/Topology_Based_Geometry_Intersections/RAY_TRIANGULATED_SURFACE_INTERSECTION.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/ANIMATED_VISUALIZATION.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_BOOL_COLOR_MAP.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_CALLBACK.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_COLOR.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_COLOR_RAMP.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_CONSTANT_COLOR_MAP.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_LIGHT.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_INDEXED_COLOR_MAP.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_SELECTION.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_TRIANGULATED_SURFACE.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_UNIFORM_SLICE.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_WORLD.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_MOUSE_HANDLER.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL/OPENGL_POINTS_3D.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL_Components/OPENGL_COMPONENT_BASIC.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL_Components/OPENGL_COMPONENT_DIAGNOSTICS.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL_Components/OPENGL_COMPONENT_FACE_SCALAR_FIELD_3D.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL/OpenGL_Components/OPENGL_COMPONENT_TRIANGULATED_SURFACE.h>
#include <PhysBAM_Rendering/PhysBAM_OpenGL_Solids/OpenGL_Rigids_Components/OPENGL_COMPONENT_RIGID_BODY_COLLECTION_3D.h>
#include <PhysBAM_Solids/PhysBAM_Rigids/Read_Write/Particles/READ_WRITE_RIGIDS_PARTICLES.h>
#include <fstream>
#include <sstream>
#include <string.h>
#include <pthread.h>
#include <time.h>
//
#include "STRO_NEW.h"
#include "opengl_callbacks_drive.h"

namespace PhysBAM {
using namespace std;
template<class T,class RW>
class INTERACTIVE_VISUALIZATION:public ANIMATED_VISUALIZATION,public OPENGL_MOUSE_HANDLER
{
    typedef VECTOR<T,3> TV;
public:
    INTERACTIVE_VISUALIZATION();
    ~INTERACTIVE_VISUALIZATION();
    int argc;
    char** argv;
    bool is_interactive;
    bool last_mouse_location_init;
    TV last_mouse_location;
    TV object_velocity;
    TV object_delta;
    TV target_position;
    T camera_distance;
    std::clock_t since_last_drag,now;

    virtual void Initialize_Components_And_Key_Bindings();
    virtual void Initialize_Lights();

protected:
    bool Handle_Click(int button,int state,int x,int y,bool ctrl_pressed,bool shift_pressed)
    {
        return ctrl_pressed;
    }

    bool Handle_Drag(int x,int y)
    {
        return opengl_world.drag_current_selection;
    }

    void Display_Follow_Object()
    {
#ifndef USE_OPENGLES
        if(!current_selection) return;
        glPushAttrib(GL_LIGHTING_BIT);
        glDisable(GL_LIGHTING);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();

        OPENGL_OBJECT* obj=Find_Component("Rigid Bodies");
        OPENGL_SELECTION_COMPONENT_RIGID_GEOMETRY_COLLECTION_3D<T> *real_selection=(OPENGL_SELECTION_COMPONENT_RIGID_GEOMETRY_COLLECTION_3D<T>*)(current_selection);
        TV tracking_location=((OPENGL_COMPONENT_RIGID_BODY_COLLECTION_3D<float> *)obj)->rigid_body_collection.Rigid_Body(real_selection->body_id).X();

        glTranslatef(tracking_location.x,tracking_location.y,tracking_location.z);
        target_position=tracking_location;
        GLfloat clear_color[4];
        glGetFloatv(GL_COLOR_CLEAR_VALUE,clear_color);
        glColor3f(1-clear_color[0],1-clear_color[1],1-clear_color[2]);
        float logcd=log(camera_distance)/log(20.);
        float smallsize=pow(20.0,floor(logcd-.5));
        glutWireCube(smallsize);
        glutWireCube(20*smallsize);
        glPopMatrix();
        glPopAttrib();
#endif
    }
    DEFINE_CALLBACK_CREATOR(INTERACTIVE_VISUALIZATION,Display_Follow_Object);

private:
    ARRAY<int> rigid_bodies_no_draw_list;
	OPENGL_COMPONENT_RIGID_BODY_COLLECTION_3D<T>* rigid_bodies_component;
public:
//#####################################################################
// Initialize_Drive_Control
// This function binds all the keys to their respective callbacks.
// (There are more in Initialize_Components_And_Key_Bindings below.)
//#####################################################################
void Initialize_Drive_Callbacks(STRO_NEW<T>* rigids_example)
{
	//opengl_world.Bind_Key('w', new OPENGL_CALLBACK_ACCL(&(rigids_example->accl), &(rigids_example->brk)));
	//opengl_world.Bind_Key('s', new OPENGL_CALLBACK_BRK(&(rigids_example->accl), &(rigids_example->brk)));
	opengl_world.Bind_Key(OPENGL_KEY::UP, new OPENGL_CALLBACK_FORWARD(&(rigids_example->forward), &(rigids_example->back), &(rigids_example->curr_accel)));
	opengl_world.Bind_Key(OPENGL_KEY::DOWN, new OPENGL_CALLBACK_BACK(&(rigids_example->forward), &(rigids_example->back), &(rigids_example->curr_accel)));
	opengl_world.Bind_Key(OPENGL_KEY::LEFT, new OPENGL_CALLBACK_TURNLEFT(&(rigids_example->dst_wise)));
	opengl_world.Bind_Key(OPENGL_KEY::RIGHT, new OPENGL_CALLBACK_TURNRIGHT(&(rigids_example->dst_wise)));
	opengl_world.Bind_Key('n', new OPENGL_CALLBACK_NITRO(&(rigids_example->nitro)));
	opengl_world.Bind_Key('b', new OPENGL_CALLBACK_REVERSE(&(rigids_example->reverse), &(rigids_example->forward), &(rigids_example->reverse_ref), &(rigids_example->curr_accel)));
	opengl_world.Bind_Key('l', new OPENGL_CALLBACK_LIFT(&(rigids_example->lift)));
	OPENGL_COMPONENT_RIGID_BODY_COLLECTION_3D<T>* rigid_bodies_component=0;
    opengl_world.Append_Bind_Key('6',rigid_bodies_component->Toggle_Draw_Mode_CB());//
}

}; // end of INTERACTIVE_VISUALIZATION class definition

//#####################################################################
// Constructor
//#####################################################################
template<class T,class RW> INTERACTIVE_VISUALIZATION<T,RW>::
INTERACTIVE_VISUALIZATION():argc(0),argv(0),is_interactive(true)
{}
//#####################################################################
// Destructor
//#####################################################################
template<class T,class RW> INTERACTIVE_VISUALIZATION<T,RW>::
~INTERACTIVE_VISUALIZATION() {
    // delete argv;
}

//#####################################################################
// Function Initialize_Lights
//#####################################################################
template<class T,class RW> void INTERACTIVE_VISUALIZATION<T,RW>::
Initialize_Lights()
{
    opengl_world.Add_Light(new OPENGL_LIGHT(VECTOR<double,3>(0,15,25),.3f));
    /*for (int i = 0; i < 4; i++) {
       OPENGL_LIGHT *light = new OPENGL_LIGHT(VECTOR<double,3>(5*i,3,0), 1.0f);
       opengl_world.lights.Append(light);
    }*/
    opengl_world.Set_Ambient_Light(0.5f);
    float difLight0[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, difLight0);
}

//#####################################################################
// Function Initialize_Components_And_Key_Bindings
//#####################################################################
template<class T,class RW> void INTERACTIVE_VISUALIZATION<T,RW>::
Initialize_Components_And_Key_Bindings()
{
    ANIMATED_VISUALIZATION::Initialize_Components_And_Key_Bindings();
    opengl_world.Set_Key_Binding_Category_Priority(1);
    opengl_world.Unbind_Keys("abBCdDEeFjJjkKlLMotTvV 1!2@3#4$5%67&89 ^=-`{}\b\\[]~\t");
    opengl_world.Set_External_Mouse_Handler(this);

    //OPENGL_COMPONENT_RIGID_BODY_COLLECTION_3D<T>* rigid_bodies_component=0;
    rigid_bodies_component=new OPENGL_COMPONENT_RIGID_BODY_COLLECTION_3D<T,RW>();
    rigid_bodies_component->Set_Vector_Size(0.01);
    rigid_bodies_component->is_interactive = is_interactive;
    rigid_bodies_component->skip_last_body=false;
    for (int i=1;i<=rigid_bodies_no_draw_list.m;i++) {
        rigid_bodies_component->Set_Draw_Object(rigid_bodies_no_draw_list(i),false);
    }
    // Read hints if available
    opengl_world.Set_Key_Binding_Category("Rigid Bodies");
    Add_Component(rigid_bodies_component,"Rigid Bodies",'\0',BASIC_VISUALIZATION::OWNED|BASIC_VISUALIZATION::SELECTABLE);
    opengl_world.Append_Bind_Key('5',rigid_bodies_component->Toggle_Draw_Mode_CB());
    opengl_world.Append_Bind_Key('%',rigid_bodies_component->Toggle_Velocity_Vectors_CB());
    opengl_world.Append_Bind_Key('a',rigid_bodies_component->Toggle_Individual_Axes_CB());
    opengl_world.Append_Bind_Key('%',rigid_bodies_component->Toggle_Show_Object_Names_CB());
    opengl_world.Append_Bind_Key('=',rigid_bodies_component->Increase_Vector_Size_CB());
    opengl_world.Append_Bind_Key('-',rigid_bodies_component->Decrease_Vector_Size_CB());
    opengl_world.Append_Bind_Key('M',rigid_bodies_component->Toggle_Draw_Particles_CB());
    opengl_world.Append_Bind_Key(OPENGL_KEY(OPENGL_KEY::F5),rigid_bodies_component->Toggle_Forces_And_Torques_CB());
    opengl_world.Append_Bind_Key('o',rigid_bodies_component->Toggle_One_Sided_CB());
    //opengl_world.Append_Bind_Key('6',rigid_bodies_component->Toggle_Draw_Mode_CB());//
     
    Selection_Priority(OPENGL_SELECTION::POINTS_3D)=100;
    Selection_Priority(OPENGL_SELECTION::COMPONENT_PARTICLES_3D)=100;
    Selection_Priority(OPENGL_SELECTION::TRIANGULATED_SURFACE_VERTEX)=90;
    Selection_Priority(OPENGL_SELECTION::TRIANGULATED_SURFACE_SEGMENT)=89;
    Selection_Priority(OPENGL_SELECTION::TRIANGULATED_SURFACE_TRIANGLE)=88;
    Selection_Priority(OPENGL_SELECTION::TETRAHEDRALIZED_VOLUME_VERTEX)=85;
    Selection_Priority(OPENGL_SELECTION::TETRAHEDRALIZED_VOLUME_TETRAHEDRON)=84;
    Selection_Priority(OPENGL_SELECTION::COMPONENT_RIGID_BODIES_3D)=80;
    Selection_Priority(OPENGL_SELECTION::SEGMENTED_CURVE_VERTEX_3D)=79;
    Selection_Priority(OPENGL_SELECTION::SEGMENTED_CURVE_SEGMENT_3D)=78;
    Selection_Priority(OPENGL_SELECTION::GRID_NODE_3D)=70;
}


}
#endif
