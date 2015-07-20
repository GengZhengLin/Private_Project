using UnityEngine;
using SimpleJSON;
using System.Collections;
using System;
using System.Runtime.InteropServices;


public class SceneController : MonoBehaviour {

	// Use this for initialization
	public GameObject jeep;
	[DllImport("game_dll")]
	static extern IntPtr API_Update_Frame ();
	[DllImport("game_dll")]
	static extern void API_Init ();
	[DllImport("game_dll")]
	static extern void API_Free_Game ();
	
	void Start () {
		API_Init ();
	}
	
	// Update is called once per frame
	void Update () {
		string s = Marshal.PtrToStringAnsi (API_Update_Frame ());
		var jv = JSONNode.Parse(s);
		float x = jv ["pos"] [0].AsFloat, y = jv ["pos"] [1].AsFloat, z = jv ["pos"] [2].AsFloat;
		jeep.transform.position = new Vector3 (x, y, z);
		float rx = jv ["ori"] [0].AsFloat, ry = jv ["ori"] [1].AsFloat, rz = jv ["ori"] [2].AsFloat;
		jeep.transform.rotation = Quaternion.Euler (rx * 180 / 3.14F, ry * 180 / 3.14F , rz * 180 / 3.14F);
	}

	void OnDestroy(){
		API_Free_Game ();
	}
}
