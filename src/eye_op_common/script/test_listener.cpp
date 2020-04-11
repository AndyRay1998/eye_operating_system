#include <python3.5/Python.h>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"



using namespace std;    //导入std名字空间

// https://www.cnblogs.com/xuyuan77/p/8419442.html

void chatterCallback1(const std_msgs::Float32MultiArray::ConstPtr& msg1)
{
  ROS_INFO("Galil hear Hyperion");
  ROS_INFO("I heard data of size: [%ld]", msg1->data.size());
  for (int i=0; i < msg1->data.size(); i++)
  {
    ROS_INFO("data: [%f]", msg1->data[i]);
  }

}

void chatterCallback2(const std_msgs::Float32MultiArray::ConstPtr& msg2)
{
  // ROS_INFO("I heard: [%f]", msg2->n);
  ROS_INFO("Galil hear Omni");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("Hyperion_data", 1000, chatterCallback1);
  ros::Subscriber sub2 = n.subscribe("Omni_data", 1000, chatterCallback2);




	cout << "Starting Test..." << endl;
	Py_Initialize();        //使用python之前，要调用Py_Initialize();这个函数进行初始化
	PyObject * pModule = NULL;    //声明变量
	PyObject * pFunc = NULL;    //声明变量
  //导入sys模块
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("sys.path.append('./')");
	cout << "------------"<<endl;
	pModule = PyImport_ImportModule("fw_kinematics");    //这里是要调用的Python文件名
	cout << pModule << endl;
  pFunc = PyObject_GetAttrString(pModule, "fw_kine"); //这里是要调用的函数名

  // 设置参数
	PyObject* args = PyTuple_New(7);       // 2个参数
	//PyObject* arg1 = PyUnicode_FromString("hello");    // 参数一设为，字符串
	PyObject* arg1 = PyLong_FromLong(0);    // 参数二设为，一个整数，用long表示
	PyObject* arg2 = PyLong_FromLong(0);    // 参数二设为，一个整数，用long表示
  PyObject* arg3 = PyLong_FromLong(0);
  PyObject* arg4 = PyLong_FromLong(0);
  PyObject* arg5 = PyLong_FromLong(78.081811);
  PyObject* arg6 = PyLong_FromLong(37.873248);
  PyObject* arg7 = PyLong_FromLong(0);
	PyTuple_SetItem(args, 0, arg1);
	PyTuple_SetItem(args, 1, arg2);
  PyTuple_SetItem(args, 2, arg3);
  PyTuple_SetItem(args, 3, arg4);
  PyTuple_SetItem(args, 4, arg5);
  PyTuple_SetItem(args, 5, arg6);
  PyTuple_SetItem(args, 6, arg7);

  // 调用函数`
	PyObject* pRet = PyObject_CallObject(pFunc, args);

	// 获取参数
	if (pRet)  // 验证是否调用成功
	{
		long result = PyLong_AsLong(pRet);
		//string result = PyUnicode_AsUTF8(pRet);     //返回字符串
		cout << "result:" << result << endl;
	}

	Py_Finalize();    //调用Py_Finalize,这个和Py_Initialize相对应的.

  ros::spin();
	return 0;
}
