using UnityEngine;
//using Windows.Kinect;

using System;
using System.Collections;
using System.Collections.Generic;

public class KinectManager : MonoBehaviour 
{
    // 传感器离地有多高(以米为单位)。
	public float sensorHeight = 1.0f;

    // Kinect仰角(角度)
	public float sensorAngle = 0f;

    // 公共Bool决定是否接收和计算用户映射。
	public bool computeUserMap = false;

    // 公共Bool决定是否接收和计算彩色地图。
	public bool computeColorMap = false;

    // 确定公共Bool是否接收和计算红外地图
	public bool computeInfraredMap = false;

    // 公共Bool决定是否在GUI上显示用户映射。
	public bool displayUserMap = false;

    // 确定是否在GUI上显示颜色映射
	public bool displayColorMap = false;

    // 确定是否在用户地图上显示骨架线
	public bool displaySkeletonLines = false;
	
	//公共浮动，用于指定深度和彩色地图所使用的图像宽度，如相机宽度的%。高度是根据宽度来计算的。
    // 如果百分比为零，则在内部计算，以匹配深度图像的选定宽度和高度
	public float DisplayMapsWidthPercent = 20f;

    // 公共Bool决定是否使用多源阅读器，如果有的话
	public bool useMultiSourceReader = false;

    // 最小的用户距离，以处理骨架数据
	public float minUserDistance = 0.5f;

    // 最大用户距离，如果有的话。0表示没有最大距离限制
	public float maxUserDistance = 0f;

    // 判断是否只检测最近的用户
	public bool detectClosestUser = true;

    // 公共Bool决定是否只使用跟踪的节点(忽略推断的节点)
	public bool ignoreInferredJoints = true;

    // 平滑参数的选择
	public enum Smoothing : int { None, Default, Medium, Aggressive }
	public Smoothing smoothing = Smoothing.Default;

    // 公共Bool确定使用附加过滤器
	public bool useBoneOrientationConstraints = true;
	//public bool useBoneOrientationsFilter = false;

    // 将由Kinect用户控制的avatarcontroller对象列表
	public List<AvatarController> avatarControllers;

    // 如果需要，对每个球员进行校正。
	public KinectGestures.Gestures playerCalibrationPose;

    // 每个玩家需要检测的手势列表。
	public List<KinectGestures.Gestures> playerCommonGestures;

    // 手势检测之间的最短时间
	public float minTimeBetweenGestures = 0.7f;

    // 动作监听器列表。他们必须实现KinectGestures.GestureListenerInterface
	public List<MonoBehaviour> gestureListeners;

    // 显示消息的GUI文本。
	public GUIText calibrationText;


    // Bool跟踪是否Kinect已经初始化
	private bool kinectInitialized = false;

    // KinectManager的单例实例
	private static KinectManager instance = null;

    // 可用的传感器接口
	private List<DepthSensorInterface> sensorInterfaces = null;
    // 主要SensorData结构
	private KinectInterop.SensorData sensorData = null;

    // 深度和用户地图
//	private KinectInterop.DepthBuffer depthImage;
//	private KinectInterop.BodyIndexBuffer bodyIndexImage;
//	private KinectInterop.UserHistogramBuffer userHistogramImage;
	private Color32[] usersHistogramImage;
	private ushort[] usersPrevState;
	private float[] usersHistogramMap;

	private Texture2D usersLblTex;
	private Rect usersMapRect;
	private int usersMapSize;
//	private int minDepth;
//	private int maxDepth;
	
	// Color map
	//private KinectInterop.ColorBuffer colorImage;
	private Texture2D usersClrTex;
	private Rect usersClrRect;
	private int usersClrSize;

    // Kinect身体帧数据
	private KinectInterop.BodyFrameData bodyFrame;
	//private Int64 lastBodyFrameTime = 0;

    // 所有用户列表
	private List<Int64> alUserIds;
	private Dictionary<Int64, int> dictUserIdToIndex;
	
	// Primary (first or closest) user ID
	private Int64 liPrimaryUserId = 0;

    // Kinect世界矩阵
	private Matrix4x4 kinectToWorld = Matrix4x4.zero;
	//private Matrix4x4 mOrient = Matrix4x4.zero;

    // 为每个玩家校准手势数据
	private Dictionary<Int64, KinectGestures.GestureData> playerCalibrationData = new Dictionary<Int64, KinectGestures.GestureData>();

    // 姿态数据和参数
	private Dictionary<Int64, List<KinectGestures.GestureData>> playerGesturesData = new Dictionary<Int64, List<KinectGestures.GestureData>>();
	private Dictionary<Int64, float> gesturesTrackingAtTime = new Dictionary<Int64, float>();

    //动作监听器列表。他们必须实现KinectGestures.GestureListenerInterface
	public List<KinectGestures.GestureListenerInterface> gestureListenerInts;

    // 身体过滤器实例
	private JointPositionsFilter jointPositionFilter = null;
	private BoneOrientationsConstraint boneConstraintsFilter = null;
	//private BoneOrientationsFilter boneOrientationFilter = null;

    // 返回单个KinectManager实例
    public static KinectManager Instance
    {
        get
        {
            return instance;
        }
    }

    // 检查Kinect是否初始化并准备使用。如果没有，则在kine -sensor初始化过程中有一个错误
	public static bool IsKinectInitialized()
	{
		return instance != null ? instance.kinectInitialized : false;
	}

    // 检查Kinect是否初始化并准备使用。如果没有，则在kine -sensor初始化过程中有一个错误
	public bool IsInitialized()
	{
		return kinectInitialized;
	}

    // 返回由传感器跟踪的尸体数量
	public int GetSensorBodyCount()
	{
		return sensorData != null ? sensorData.bodyCount : 0;
	}

    // 返回传感器跟踪的关节数量
	public int GetSensorJointCount()
	{
		return sensorData != null ? sensorData.jointCount : 0;
	}

    // 返回关节数组中给定关节的索引
	public int GetJointIndex(KinectInterop.JointType joint)
	{
		if(sensorData != null && sensorData.sensorInterface != null)
		{
			return sensorData.sensorInterface.GetJointIndex(joint);
		}
		
		// fallback - index matches the joint
		return (int)joint;
	}

    // 返回给定索引处的关节
	public KinectInterop.JointType GetJointAtIndex(int index)
	{
		if(sensorData != null && sensorData.sensorInterface != null)
		{
			return sensorData.sensorInterface.GetJointAtIndex(index);
		}
		
		// fallback - index matches the joint
		return (KinectInterop.JointType)index;
	}

    // 返回给定关节的父关节
	public KinectInterop.JointType GetParentJoint(KinectInterop.JointType joint)
	{
		if(sensorData != null && sensorData.sensorInterface != null)
		{
			return sensorData.sensorInterface.GetParentJoint(joint);
		}

        // 后退-返回相同的关节(即末端关节)
		return joint;
	}

    // 返回传感器支持的彩色图像的宽度。
	public int GetColorImageWidth()
	{
		return sensorData != null ? sensorData.colorImageWidth : 0;
	}

    // 返回传感器支持的彩色图像的高度
	public int GetColorImageHeight()
	{
		return sensorData != null ? sensorData.colorImageHeight : 0;
	}

    // 返回传感器支持的深度图像的宽度
	public int GetDepthImageWidth()
	{
		return sensorData != null ? sensorData.depthImageWidth : 0;
	}

    // 返回传感器支持的深度图像的高度
	public int GetDepthImageHeight()
	{
		return sensorData != null ? sensorData.depthImageHeight : 0;
	}

    // 如果ComputeUserMap为真，则返回原始深度/用户数据
	public ushort[] GetRawDepthMap()
	{
		return sensorData != null ? sensorData.depthImage : null;
	}

    // 返回原始红外数据，如果ComputeInfraredMap为真
	public ushort[] GetRawInfraredMap()
	{
		return sensorData != null ? sensorData.infraredImage : null;
	}


    // 如果ComputeUserMap为真，返回深度图像/用户直方图纹理
    public Texture2D GetUsersLblTex()
    { 
		return usersLblTex;
	}

    // 返回颜色图像纹理，如果ComputeColorMap为真
	public Texture2D GetUsersClrTex()
	{ 
		return usersClrTex;
	}

	// 如果传感器当前检测到至少一个用户，则返回true
	public bool IsUserDetected()
	{
		return kinectInitialized && (alUserIds.Count > 0);
	}

    // 如果用户被校准并准备使用，则返回true
	public bool IsUserCalibrated(Int64 userId)
	{
		return dictUserIdToIndex.ContainsKey(userId);
	}

    // 返回当前检测到的用户数量
	public int GetUsersCount()
	{
		return alUserIds.Count;
	}

    // 通过给定的索引返回UserID
	public Int64 GetUserIdByIndex(int i)
	{
		if(i >= 0 && i < alUserIds.Count)
		{
			return alUserIds[i];
		}
		
		return 0;
	}

    //返回主用户的UserID(第一个或最近的用户)，如果有的话
	public Int64 GetPrimaryUserID()
	{
		return liPrimaryUserId;
	}

    // 设置新的主用户ID，以更改活动用户。
	public bool SetPrimaryUserID(Int64 userId)
	{
		bool bResult = false;

		if(alUserIds.Contains(userId) || (userId == 0))
		{
			liPrimaryUserId = userId;
			bResult = true;
		}

		return bResult;
	}

    // 返回用户主体数据，仅用于调试目的
    // 不直接改变结构中的数据吗
	public KinectInterop.BodyData GetUserBodyData(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount)
			{
				return bodyFrame.bodyData[index];
			}
		}
		
		return new KinectInterop.BodyData();
	}

    // 返回用户位置，相对于动作传感器，以米为单位
	public Vector3 GetUserPosition(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return bodyFrame.bodyData[index].position;
			}
		}
		
		return Vector3.zero;
	}

    // 返回相对于动作传感器的用户旋转
	public Quaternion GetUserOrientation(Int64 userId, bool flip)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
			   bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(flip)
					return bodyFrame.bodyData[index].normalRotation;
				else
					return bodyFrame.bodyData[index].mirroredRotation;
			}
		}
		
		return Quaternion.identity;
	}

    // 返回指定用户给定关节的原始跟踪状态
	public KinectInterop.TrackingState GetJointTrackingState(Int64 userId, int joint)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(joint >= 0 && joint < sensorData.jointCount)
				{
					return  bodyFrame.bodyData[index].joint[joint].trackingState;
				}
			}
		}
		
		return KinectInterop.TrackingState.NotTracked;
	}

    // 如果正在跟踪指定用户的给定关节，则返回true
	public bool IsJointTracked(Int64 userId, int joint)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(joint >= 0 && joint < sensorData.jointCount)
				{
					KinectInterop.JointData jointData = bodyFrame.bodyData[index].joint[joint];
					
					return ignoreInferredJoints ? (jointData.trackingState == KinectInterop.TrackingState.Tracked) : 
						(jointData.trackingState != KinectInterop.TrackingState.NotTracked);
				}
			}
		}
		
		return false;
	}

    // 返回指定用户的联合位置，相对于传感器，以米为单位。
	public Vector3 GetJointKinectPosition(Int64 userId, int joint)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
			   bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(joint >= 0 && joint < sensorData.jointCount)
				{
					KinectInterop.JointData jointData = bodyFrame.bodyData[index].joint[joint];
					return jointData.kinectPos;
				}
			}
		}
		
		return Vector3.zero;
	}

    // 返回指定用户的联合位置，相对于传感器，以米为单位。
	public Vector3 GetJointPosition(Int64 userId, int joint)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(joint >= 0 && joint < sensorData.jointCount)
				{
					KinectInterop.JointData jointData = bodyFrame.bodyData[index].joint[joint];
					return jointData.position;
				}
			}
		}
		
		return Vector3.zero;
	}

    // 返回指定用户的关节方向，相对于父关节
	public Vector3 GetJointDirection(Int64 userId, int joint, bool flipX, bool flipZ)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(joint >= 0 && joint < sensorData.jointCount)
				{
					KinectInterop.JointData jointData = bodyFrame.bodyData[index].joint[joint];
					Vector3 jointDir = jointData.direction;

					if(flipX)
						jointDir.x = -jointDir.x;
					
					if(flipZ)
						jointDir.z = -jointDir.z;
					
					return jointDir;
				}
			}
		}
		
		return Vector3.zero;
	}

    // 返回指定用户的firstJoint和secondJoint之间的方向
	public Vector3 GetDirectionBetweenJoints(Int64 userId, int firstJoint, int secondJoint, bool flipX, bool flipZ)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				KinectInterop.BodyData bodyData = bodyFrame.bodyData[index];
				
				if(firstJoint >= 0 && firstJoint < sensorData.jointCount &&
					secondJoint >= 0 && secondJoint < sensorData.jointCount)
				{
					Vector3 firstJointPos = bodyData.joint[firstJoint].position;
					Vector3 secondJointPos = bodyData.joint[secondJoint].position;
					Vector3 jointDir = secondJointPos - firstJointPos;

					if(flipX)
						jointDir.x = -jointDir.x;
					
					if(flipZ)
						jointDir.z = -jointDir.z;
					
					return jointDir;
				}
			}
		}
		
		return Vector3.zero;
	}

    // 返回指定用户相对于动作传感器的关节旋转
	public Quaternion GetJointOrientation(Int64 userId, int joint, bool flip)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
			   bodyFrame.bodyData[index].bIsTracked != 0)
			{
				if(flip)
					return bodyFrame.bodyData[index].joint[joint].normalRotation;
				else
					return bodyFrame.bodyData[index].joint[joint].mirroredRotation;
			}
		}
		
		return Quaternion.identity;
	}

    // 返回指定用户相对于动作传感器的关节旋转
    // 如果置信值高，则返回true;如果置信值低，则返回false;如果没有找到用户，则返回false
	public bool IsLeftHandConfidenceHigh(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return (bodyFrame.bodyData[index].leftHandConfidence == KinectInterop.TrackingConfidence.High);
			}
		}
		
		return false;
	}

    // 检查用户的右手置信度是否高
    // 如果置信值高，则返回true;如果置信值低，则返回false;如果没有找到用户，则返回false
	public bool IsRightHandConfidenceHigh(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return (bodyFrame.bodyData[index].rightHandConfidence == KinectInterop.TrackingConfidence.High);
			}
		}
		
		return false;
	}

    // 返回用户的左手状态
	public KinectInterop.HandState GetLeftHandState(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return bodyFrame.bodyData[index].leftHandState;
			}
		}
		
		return KinectInterop.HandState.NotTracked;
	}

    // 返回用户的右手状态
	public KinectInterop.HandState GetRightHandState(Int64 userId)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				return bodyFrame.bodyData[index].rightHandState;
			}
		}
		
		return KinectInterop.HandState.NotTracked;
	}

    // 返回指定用户的左侧的交互框，以米为单位。
	public bool GetLeftHandInteractionBox(Int64 userId, ref Vector3 leftBotBack, ref Vector3 rightTopFront, bool bValidBox)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				KinectInterop.BodyData bodyData = bodyFrame.bodyData[index];
				bool bResult = true;
				
				if(bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].trackingState == KinectInterop.TrackingState.Tracked &&
				   bodyData.joint[(int)KinectInterop.JointType.HipLeft].trackingState == KinectInterop.TrackingState.Tracked)
				{
					rightTopFront.x = bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].position.x;
					leftBotBack.x = rightTopFront.x - 2 * (rightTopFront.x - bodyData.joint[(int)KinectInterop.JointType.HipLeft].position.x);
				}
				else
				{
					bResult = bValidBox;
				}
					
				if(bodyData.joint[(int)KinectInterop.JointType.HipRight].trackingState == KinectInterop.TrackingState.Tracked &&
				   bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].trackingState == KinectInterop.TrackingState.Tracked)
				{
					leftBotBack.y = bodyData.joint[(int)KinectInterop.JointType.HipRight].position.y;
					rightTopFront.y = bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].position.y;
					
					float fDelta = (rightTopFront.y - leftBotBack.y) * 0.35f; // * 2 / 3;
					leftBotBack.y += fDelta;
					rightTopFront.y += fDelta;
				}
				else
				{
					bResult = bValidBox;
				}
					
				if(bodyData.joint[(int)KinectInterop.JointType.SpineBase].trackingState == KinectInterop.TrackingState.Tracked)
				{
					leftBotBack.z = bodyData.joint[(int)KinectInterop.JointType.SpineBase].position.z;
					rightTopFront.z = leftBotBack.z - 0.5f;
				}
				else
				{
					bResult = bValidBox;
				}
				
				return bResult;
			}
		}
		
		return false;
	}

    // 返回指定用户右手的交互框，单位为米
	public bool GetRightHandInteractionBox(Int64 userId, ref Vector3 leftBotBack, ref Vector3 rightTopFront, bool bValidBox)
	{
		if(dictUserIdToIndex.ContainsKey(userId))
		{
			int index = dictUserIdToIndex[userId];
			
			if(index >= 0 && index < sensorData.bodyCount && 
				bodyFrame.bodyData[index].bIsTracked != 0)
			{
				KinectInterop.BodyData bodyData = bodyFrame.bodyData[index];
				bool bResult = true;
				
				if(bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].trackingState == KinectInterop.TrackingState.Tracked &&
				   bodyData.joint[(int)KinectInterop.JointType.HipRight].trackingState == KinectInterop.TrackingState.Tracked)
				{
					leftBotBack.x = bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].position.x;
					rightTopFront.x = leftBotBack.x + 2 * (bodyData.joint[(int)KinectInterop.JointType.HipRight].position.x - leftBotBack.x);
				}
				else
				{
					bResult = bValidBox;
				}
					
				if(bodyData.joint[(int)KinectInterop.JointType.HipLeft].trackingState == KinectInterop.TrackingState.Tracked &&
				   bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].trackingState == KinectInterop.TrackingState.Tracked)
				{
					leftBotBack.y = bodyData.joint[(int)KinectInterop.JointType.HipLeft].position.y;
					rightTopFront.y = bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].position.y;
					
					float fDelta = (rightTopFront.y - leftBotBack.y) * 0.35f; // * 2 / 3;
					leftBotBack.y += fDelta;
					rightTopFront.y += fDelta;
				}
				else
				{
					bResult = bValidBox;
				}
					
				if(bodyData.joint[(int)KinectInterop.JointType.SpineBase].trackingState == KinectInterop.TrackingState.Tracked)
				{
					leftBotBack.z = bodyData.joint[(int)KinectInterop.JointType.SpineBase].position.z;
					rightTopFront.z = leftBotBack.z - 0.5f;
				}
				else
				{
					bResult = bValidBox;
				}
				
				return bResult;
			}
		}
		
		return false;
	}

    // 如果ComputeUserMap为真，则返回特定像素的深度数据
	public ushort GetDepthForPixel(int x, int y)
	{
		if(sensorData != null && sensorData.depthImage != null)
		{
			int index = y * sensorData.depthImageWidth + x;
			
			if(index >= 0 && index < sensorData.depthImage.Length)
			{
				return sensorData.depthImage[index];
			}
		}

		return 0;
	}
	
	// returns 3d coordinates of a depth-map point, or Vector3.zero if the sensor is not initialized
	public Vector3 MapDepthPointToSpaceCoords(Vector2 posPoint, ushort depthValue, bool bWorldCoords)
	{
		Vector3 posKinect = Vector3.zero;
		
		if(kinectInitialized)
		{
			posKinect = KinectInterop.MapDepthPointToSpaceCoords(sensorData, posPoint, depthValue);
			
			if(bWorldCoords)
			{
				posKinect = kinectToWorld.MultiplyPoint3x4(posKinect);
			}
		}
		
		return posKinect;
	}

    // 返回3D点或Vector2的深度映射坐标。如果Kinect没有初始化，则为0
	public Vector2 MapSpacePointToDepthCoords(Vector3 posPoint)
	{
		Vector2 posDepth = Vector2.zero;
		
		if(kinectInitialized)
		{
			posDepth = KinectInterop.MapSpacePointToDepthCoords(sensorData, posPoint);
		}
		
		return posDepth;
	}

    // 返回给定深度点的颜色映射坐标
	public Vector2 MapDepthPointToColorCoords(Vector2 posPoint, ushort depthValue)
	{
		Vector2 posColor = Vector3.zero;
		
		if(kinectInitialized)
		{
			posColor = KinectInterop.MapDepthPointToColorCoords(sensorData, posPoint, depthValue);
		}
		
		return posColor;
	}

    // 删除当前检测到的kinect用户，允许启动新的检测/校准过程
	public void ClearKinectUsers()
	{
		if(!kinectInitialized)
			return;

		// remove current users
		for(int i = alUserIds.Count - 1; i >= 0; i--)
		{
			Int64 userId = alUserIds[i];
			RemoveUser(userId);
		}
		
		ResetFilters();
	}

    // 重置数据过滤器
	public void ResetFilters()
	{
		if(jointPositionFilter != null)
		{
			jointPositionFilter.Reset();
		}
	}

    // 向指定用户检测到的手势列表中添加一个手势
	public void DetectGesture(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : new List<KinectGestures.GestureData>();
		int index = GetGestureIndex(gesture, ref gesturesData);

		if(index >= 0)
		{
			DeleteGesture(UserId, gesture);
		}
		
		KinectGestures.GestureData gestureData = new KinectGestures.GestureData();
		
		gestureData.userId = UserId;
		gestureData.gesture = gesture;
		gestureData.state = 0;
		gestureData.joint = 0;
		gestureData.progress = 0f;
		gestureData.complete = false;
		gestureData.cancelled = false;
		
		gestureData.checkForGestures = new List<KinectGestures.Gestures>();
		switch(gesture)
		{
		case KinectGestures.Gestures.ZoomIn:
			gestureData.checkForGestures.Add(KinectGestures.Gestures.ZoomOut);
			gestureData.checkForGestures.Add(KinectGestures.Gestures.Wheel);			
			break;
			
		case KinectGestures.Gestures.ZoomOut:
			gestureData.checkForGestures.Add(KinectGestures.Gestures.ZoomIn);
			gestureData.checkForGestures.Add(KinectGestures.Gestures.Wheel);			
			break;
			
		case KinectGestures.Gestures.Wheel:
			gestureData.checkForGestures.Add(KinectGestures.Gestures.ZoomIn);
			gestureData.checkForGestures.Add(KinectGestures.Gestures.ZoomOut);			
			break;
		}

		gesturesData.Add(gestureData);
		playerGesturesData[UserId] = gesturesData;
		
		if(!gesturesTrackingAtTime.ContainsKey(UserId))
		{
			gesturesTrackingAtTime[UserId] = 0f;
		}
	}

    // 为指定用户的给定手势重置手势数据状态
	public bool ResetGesture(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;
		if(index < 0)
			return false;
		
		KinectGestures.GestureData gestureData = gesturesData[index];
		
		gestureData.state = 0;
		gestureData.joint = 0;
		gestureData.progress = 0f;
		gestureData.complete = false;
		gestureData.cancelled = false;
		gestureData.startTrackingAtTime = Time.realtimeSinceStartup + KinectInterop.Constants.MinTimeBetweenSameGestures;

		gesturesData[index] = gestureData;
		playerGesturesData[UserId] = gesturesData;

		return true;
	}

    // 为指定用户的所有检测到的手势重新设置gesture-data状态。
	public void ResetPlayerGestures(Int64 UserId)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			int listSize = gesturesData.Count;
			
			for(int i = 0; i < listSize; i++)
			{
				ResetGesture(UserId, gesturesData[i].gesture);
			}
		}
	}

    // 从指定用户检测到的手势列表中删除给定的手势
	public bool DeleteGesture(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;
		if(index < 0)
			return false;
		
		gesturesData.RemoveAt(index);
		playerGesturesData[UserId] = gesturesData;

		return true;
	}

    // 清除指定用户检测到的手势列表
	public void ClearGestures(Int64 UserId)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			gesturesData.Clear();
			playerGesturesData[UserId] = gesturesData;
		}
	}

    // 返回指定用户检测到的手势列表中检测到的手势的计数
	public int GetGesturesCount(Int64 UserId)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			return gesturesData.Count;
		}

		return 0;
	}

    // 返回指定用户检测到的手势的列表
	public List<KinectGestures.Gestures> GetGesturesList(Int64 UserId)
	{
		List<KinectGestures.Gestures> list = new List<KinectGestures.Gestures>();
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			foreach(KinectGestures.GestureData data in gesturesData)
				list.Add(data.gesture);
		}

		return list;
	}

    // 如果给定的手势在指定用户检测到的手势列表中，则返回true
	public bool IsGestureDetected(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		return index >= 0;
	}

    // 如果指定用户的给定手势已完成，则返回true
	public bool IsGestureComplete(Int64 UserId, KinectGestures.Gestures gesture, bool bResetOnComplete)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		if(index >= 0)
		{
			KinectGestures.GestureData gestureData = gesturesData[index];
			
			if(bResetOnComplete && gestureData.complete)
			{
				ResetPlayerGestures(UserId);
				return true;
			}
			
			return gestureData.complete;
		}
		
		return false;
	}

    // 如果指定用户的给定手势被取消，则返回true
	public bool IsGestureCancelled(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		if(index >= 0)
		{
			KinectGestures.GestureData gestureData = gesturesData[index];
			return gestureData.cancelled;
		}
		
		return false;
	}

    // 返回指定用户的给定手势的范围[0,1]的进展
	public float GetGestureProgress(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		if(index >= 0)
		{
			KinectGestures.GestureData gestureData = gesturesData[index];
			return gestureData.progress;
		}
		
		return 0f;
	}

    // 为指定的用户返回给定手势的当前“屏幕位置”
	public Vector3 GetGestureScreenPos(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		if(index >= 0)
		{
			KinectGestures.GestureData gestureData = gesturesData[index];
			return gestureData.screenPos;
		}
		
		return Vector3.zero;
	}
	
	// KinectManager's Internal Methods
	
	void Awake()
	{
		try
		{
            // 初始化可用的传感器接口
			bool bNeedRestart = false;
			sensorInterfaces = KinectInterop.InitSensorInterfaces(ref bNeedRestart);

			if(bNeedRestart)
			{
                // 重新加载相同的水平
				Application.LoadLevel(Application.loadedLevel);
			}
		} 
		catch (Exception ex) 
		{
			Debug.LogError(ex.ToString());
			
			if(calibrationText != null)
			{
				calibrationText.GetComponent<GUIText>().text = ex.Message;
			}
		}
	}

	void Start() 
	{
		try
		{
            // 尝试初始化默认的Kinect2传感器
			KinectInterop.FrameSource dwFlags = KinectInterop.FrameSource.TypeBody;
			if(computeUserMap)
				dwFlags |= KinectInterop.FrameSource.TypeDepth | KinectInterop.FrameSource.TypeBodyIndex;
			if(computeColorMap)
				dwFlags |= KinectInterop.FrameSource.TypeColor;
			if(computeInfraredMap)
				dwFlags |= KinectInterop.FrameSource.TypeInfrared;

            // 打开默认的传感器
			sensorData = KinectInterop.OpenDefaultSensor(sensorInterfaces, dwFlags, sensorAngle, useMultiSourceReader);
			if (sensorData == null)
			{
				throw new Exception("OpenDefaultSensor failed");
			}

            //创造转换矩阵- kinect到世界
			Quaternion quatTiltAngle = new Quaternion();
			quatTiltAngle.eulerAngles = new Vector3(-sensorAngle, 0.0f, 0.0f);
			
			kinectToWorld.SetTRS(new Vector3(0.0f, sensorHeight, 0.0f), quatTiltAngle, Vector3.one);
		}
		catch(DllNotFoundException ex)
		{
			string message = ex.Message + " cannot be loaded. Please check the Kinect SDK installation.";
			
			Debug.LogError(message);
			Debug.LogException(ex);
			
			if(calibrationText != null)
			{
				calibrationText.GetComponent<GUIText>().text = message;
			}
			
			return;
		}
		catch(Exception ex)
		{
			string message = ex.Message;

			Debug.LogError(message);
			Debug.LogException(ex);
			
			if(calibrationText != null)
			{
				calibrationText.GetComponent<GUIText>().text = message;
			}
			
			return;
		}

        // 设置单例实例
		instance = this;

        // init骨架结构
		bodyFrame = new KinectInterop.BodyFrameData(sensorData.bodyCount, KinectInterop.Constants.JointCount); // sensorData.jointCount

		KinectInterop.SmoothParameters smoothParameters = new KinectInterop.SmoothParameters();
		
		switch(smoothing)
		{
			case Smoothing.Default:
				smoothParameters.smoothing = 0.5f;
				smoothParameters.correction = 0.5f;
				smoothParameters.prediction = 0.5f;
				smoothParameters.jitterRadius = 0.05f;
				smoothParameters.maxDeviationRadius = 0.04f;
				break;
			case Smoothing.Medium:
				smoothParameters.smoothing = 0.5f;
				smoothParameters.correction = 0.1f;
				smoothParameters.prediction = 0.5f;
				smoothParameters.jitterRadius = 0.1f;
				smoothParameters.maxDeviationRadius = 0.1f;
				break;
			case Smoothing.Aggressive:
				smoothParameters.smoothing = 0.7f;
				smoothParameters.correction = 0.3f;
				smoothParameters.prediction = 1.0f;
				smoothParameters.jitterRadius = 1.0f;
				smoothParameters.maxDeviationRadius = 1.0f;
				break;
		}

        // 初始化数据过滤器
		jointPositionFilter = new JointPositionsFilter();
		jointPositionFilter.Init(smoothParameters);

        // 初始化骨取向约束
		if(useBoneOrientationConstraints)
		{
			boneConstraintsFilter = new BoneOrientationsConstraint();
			boneConstraintsFilter.AddDefaultConstraints();
			//boneConstraintsFilter.SetDebugText(calibrationText);
		}

        // 获取主相机矩形
		Rect cameraRect = Camera.main.pixelRect;

        // 如果需要，按百分比计算地图宽度和高度
		if(DisplayMapsWidthPercent == 0f)
		{
			DisplayMapsWidthPercent = (sensorData.depthImageWidth / 2) * 100 / cameraRect.width;
		}
		
		if(computeUserMap)
		{
			float displayMapsWidthPercent = DisplayMapsWidthPercent / 100f;
			float displayMapsHeightPercent = displayMapsWidthPercent * sensorData.depthImageHeight / sensorData.depthImageWidth;
			
			float displayWidth = cameraRect.width * displayMapsWidthPercent;
			float displayHeight = cameraRect.width * displayMapsHeightPercent;

            // 初始化深度和标签映射相关的东西
	        usersLblTex = new Texture2D(sensorData.depthImageWidth, sensorData.depthImageHeight);
			usersMapRect = new Rect(cameraRect.width - displayWidth, cameraRect.height, displayWidth, -displayHeight);

			usersMapSize = sensorData.depthImageWidth * sensorData.depthImageHeight;
			usersHistogramImage = new Color32[usersMapSize];
			usersPrevState = new ushort[usersMapSize];
	        usersHistogramMap = new float[5001];
		}
		
		if(computeColorMap)
		{
			float displayMapsWidthPercent = DisplayMapsWidthPercent / 100f;
			float displayMapsHeightPercent = displayMapsWidthPercent * sensorData.colorImageHeight / sensorData.colorImageWidth;
			
			float displayWidth = cameraRect.width * displayMapsWidthPercent;
			float displayHeight = cameraRect.width * displayMapsHeightPercent;

            // 初始化彩色地图相关的东西
			usersClrTex = new Texture2D(sensorData.colorImageWidth, sensorData.colorImageHeight, TextureFormat.RGBA32, false);
			usersClrRect = new Rect(cameraRect.width - displayWidth, cameraRect.height, displayWidth, -displayHeight);
			usersClrSize = sensorData.colorImageWidth * sensorData.colorImageHeight;
			
//			if(computeUserMap && displayColorMap)
//			{
//				usersMapRect.x -= cameraRect.width * displayMapsWidthPercent;
//			}
		}

        // 试着在现场自动找到可用的化身控制器
		if(avatarControllers.Count == 0)
		{
			AvatarController[] avatars = FindObjectsOfType(typeof(AvatarController)) as AvatarController[];
			
			foreach(AvatarController avatar in avatars)
			{
				avatarControllers.Add(avatar);
			}
		}

        // 初始化用户列表以包含所有用户。
        alUserIds = new List<Int64>();
        dictUserIdToIndex = new Dictionary<Int64, int>();
	
		kinectInitialized = true;
		DontDestroyOnLoad(gameObject);
		
		// GUI Text.
		if(calibrationText != null)
		{
			calibrationText.GetComponent<GUIText>().text = "WAITING FOR USERS";
		}
		
		Debug.Log("Waiting for users.");
	}
	
	void OnApplicationQuit()
	{
        // 退出时关闭Kinect。
		if(kinectInitialized)
		{
			KinectInterop.CloseSensor(sensorData);
			
//			KinectInterop.ShutdownKinectSensor();

			instance = null;
		}
	}
	
    void OnGUI()
    {
		if(kinectInitialized)
		{
	        if(computeUserMap && displayUserMap)
	        {
	            GUI.DrawTexture(usersMapRect, usersLblTex);
	        }
			else if(computeColorMap && displayColorMap)
			{
				GUI.DrawTexture(usersClrRect, usersClrTex);
			}
		}
    }
	
	void Update() 
	{
		if(kinectInitialized)
		{
			KinectInterop.UpdateSensorData(sensorData);

			if(useMultiSourceReader)
			{
				KinectInterop.GetMultiSourceFrame(sensorData);
			}

			if(computeColorMap)
			{
				if(KinectInterop.PollColorFrame(sensorData))
				{
					UpdateColorMap();
				}
			}
			
			if(computeUserMap)
			{
				if(KinectInterop.PollDepthFrame(sensorData))
				{
					UpdateUserMap();
				}
			}
			
			if(computeInfraredMap)
			{
				if(KinectInterop.PollInfraredFrame(sensorData))
				{
					UpdateInfraredMap();
				}
			}
			
			if(KinectInterop.PollBodyFrame(sensorData, ref bodyFrame, ref kinectToWorld))
			{
				//lastFrameTime = bodyFrame.liRelativeTime;

                // 过滤跟踪的关节位置
				if(smoothing != Smoothing.None)
				{
					jointPositionFilter.UpdateFilter(ref bodyFrame);
				}

				ProcessBodyFrameData();
			}

			if(useMultiSourceReader)
			{
				KinectInterop.FreeMultiSourceFrame(sensorData);
			}
			
			foreach (AvatarController controller in avatarControllers)
			{
				int userIndex = controller.playerIndex;

				if((userIndex >= 0) && (userIndex < alUserIds.Count))
				{
					Int64 userId = alUserIds[userIndex];
					controller.UpdateAvatar(userId);
				}
			}
				
			
			foreach(Int64 userId in alUserIds)
			{
				if(!playerGesturesData.ContainsKey(userId))
					continue;

                // 检查玩家1的手势
				CheckForGestures(userId);

                // 检查完整的手势
				List<KinectGestures.GestureData> gesturesData = playerGesturesData[userId];
				
				foreach(KinectGestures.GestureData gestureData in gesturesData)
				{
					if(gestureData.complete)
					{
//						if(gestureData.gesture == KinectGestures.Gestures.Click)
//						{
//							if(controlMouseCursor)
//							{
//								MouseControl.MouseClick();
//							}
//						}
				
						foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
						{
							if(listener.GestureCompleted(userId, 0, gestureData.gesture, (KinectInterop.JointType)gestureData.joint, gestureData.screenPos))
							{
								ResetPlayerGestures(userId);
							}
						}
					}
					else if(gestureData.cancelled)
					{
						foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
						{
							if(listener.GestureCancelled(userId, 0, gestureData.gesture, (KinectInterop.JointType)gestureData.joint))
							{
								ResetGesture(userId, gestureData.gesture);
							}
						}
					}
					else if(gestureData.progress >= 0.1f)
					{
//						if((gestureData.gesture == KinectGestures.Gestures.RightHandCursor || 
//						    gestureData.gesture == KinectGestures.Gestures.LeftHandCursor) && 
//						   gestureData.progress >= 0.5f)
//						{
//							if(handCursor != null)
//							{
//								handCursor.transform.position = Vector3.Lerp(handCursor.transform.position, gestureData.screenPos, 3 * Time.deltaTime);
//							}
//							
//							if(controlMouseCursor)
//							{
//								MouseControl.MouseMove(gestureData.screenPos);
//							}
//						}
						
						foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
						{
							listener.GestureInProgress(userId, 0, gestureData.gesture, gestureData.progress, 
							                           (KinectInterop.JointType)gestureData.joint, gestureData.screenPos);
						}
					}
				}
			}
			
		}
	}

    // 更新彩色图像
	void UpdateColorMap()
	{
		//usersClrTex.SetPixels32(colorImage.pixels);
		usersClrTex.LoadRawTextureData(sensorData.colorImage);
		usersClrTex.Apply();
	}

    // 更新用户直方图
    void UpdateUserMap()
    {
		//if(KinectInterop.PollUserHistogramFrame(ref userHistogramImage, computeColorMap))
		{
            // 画用户直方图
			//usersLblTex.SetPixels32(userHistogramImage.pixels);

			UpdateUserHistogramImage();
			usersLblTex.SetPixels32(usersHistogramImage);

            // 画轮廓线
			if(displaySkeletonLines)
			{
				for(int i = 0; i < alUserIds.Count; i++)
				{
					Int64 liUserId = alUserIds[i];
					int index = dictUserIdToIndex[liUserId];
					
					if(index >= 0 && index < sensorData.bodyCount)
					{
						DrawSkeleton(usersLblTex, ref bodyFrame.bodyData[index]);
					}
				}
			}

			usersLblTex.Apply();
		}
    }

    // 更新用户红外地图。
	void UpdateInfraredMap()
	{
        // 现在什么也不做
	}

    //更新用户直方图地图
	void UpdateUserHistogramImage()
	{
		int numOfPoints = 0;
		Array.Clear(usersHistogramMap, 0, usersHistogramMap.Length);

        //计算深度的累积直方图
		for (int i = 0; i < usersMapSize; i++)
		{
            // 只计算包含用户的深度
			if (sensorData.bodyIndexImage[i] != 255)
			{
				ushort depth = sensorData.depthImage[i];
				if(depth > 5000)
					depth = 5000;

				usersHistogramMap[depth]++;
				numOfPoints++;
			}
		}
		
		if (numOfPoints > 0)
		{
			for (int i = 1; i < usersHistogramMap.Length; i++)
			{   
				usersHistogramMap[i] += usersHistogramMap[i - 1];
			}
			
			for (int i = 0; i < usersHistogramMap.Length; i++)
			{
				usersHistogramMap[i] = 1.0f - (usersHistogramMap[i] / numOfPoints);
			}
		}

		Vector2[] colorCoords = null;
		//ColorSpacePoint[] colorCoords = null;
		if(sensorData.colorImage != null)
		{
			colorCoords = new Vector2[sensorData.depthImageWidth * sensorData.depthImageHeight];

			if(!KinectInterop.MapDepthFrameToColorCoords(sensorData, ref colorCoords))
			{
				colorCoords = null;
			}
		}

        // 根据标签贴图和深度直方图创建实际的用户纹理
		Color32 clrClear = Color.clear;
		for (int i = 0; i < usersMapSize; i++)
		{
			ushort userMap = sensorData.bodyIndexImage[i];
			ushort userDepth = sensorData.depthImage[i];

			if(userDepth > 5000)
				userDepth = 5000;
			
			ushort nowUserPixel = userMap != 255 ? (ushort)((userMap << 13) | userDepth) : userDepth;
			ushort wasUserPixel = usersPrevState[i];

            // 只绘制已更改的像素
			if(nowUserPixel != wasUserPixel)
			{
				usersPrevState[i] = nowUserPixel;
				
				if (userMap == 255)
				{
					usersHistogramImage[i] = clrClear;
				}
				else
				{
					if(sensorData.colorImage != null)
					{
						Vector2 vColorPos = Vector2.zero;

						if(colorCoords != null)
						{
							vColorPos.x = colorCoords[i].x;
							vColorPos.y = colorCoords[i].y;
						}
						else
						{
							Vector2 vDepthPos = Vector2.zero;
							vDepthPos.x = i % sensorData.depthImageWidth;
							vDepthPos.y = i / sensorData.depthImageWidth;

							vColorPos = KinectInterop.MapDepthPointToColorCoords(sensorData, vDepthPos, userDepth);
						}

						if(!float.IsInfinity(vColorPos.x) && !float.IsInfinity(vColorPos.y))
						{
							int cx = (int)vColorPos.x;
							int cy = (int)vColorPos.y;
							int colorIndex = cx + cy * sensorData.colorImageWidth;

							if(colorIndex >= 0 && colorIndex < usersClrSize)
							{
								int ci = colorIndex << 2;
								Color32 colorPixel = new Color32(sensorData.colorImage[ci], sensorData.colorImage[ci + 1], sensorData.colorImage[ci + 2], 230);
								
								usersHistogramImage[i] = colorPixel;
							}
						}
					}
					else
					{
						// Create a blending color based on the depth histogram
						float histDepth = usersHistogramMap[userDepth];
						Color c = new Color(histDepth, histDepth, histDepth, 0.9f);
						
						switch(userMap % 4)
						{
						case 0:
							usersHistogramImage[i] = Color.red * c;
							break;
						case 1:
							usersHistogramImage[i] = Color.green * c;
							break;
						case 2:
							usersHistogramImage[i] = Color.blue * c;
							break;
						case 3:
							usersHistogramImage[i] = Color.magenta * c;
							break;
						}
					}
				}
				
			}
		}
		
	}

    // 车身骨架数据进行处理
	private void ProcessBodyFrameData()
	{
		List<Int64> addedUsers = new List<Int64>();
		List<int> addedIndexes = new List<int>();

		List<Int64> lostUsers = new List<Int64>();
		lostUsers.AddRange(alUserIds);
		
		for(int i = 0; i < sensorData.bodyCount; i++)
		{
			KinectInterop.BodyData bodyData = bodyFrame.bodyData[i];
			Int64 userId = bodyData.liTrackingID;
			
			if(bodyData.bIsTracked != 0 && Mathf.Abs(bodyData.position.z) >= minUserDistance &&
			   (maxUserDistance <= 0f || Mathf.Abs(bodyData.position.z) <= maxUserDistance))
			{
                //把身体位置
				Vector3 bodyPos = bodyData.position;
				
				if(liPrimaryUserId == 0)
				{
                    // 检查这是否是最近的用户
					bool bClosestUser = true;
					int iClosestUserIndex = i;
					
					if(detectClosestUser)
					{
						for(int j = 0; j < sensorData.bodyCount; j++)
						{
							if(j != i)
							{
								KinectInterop.BodyData bodyDataOther = bodyFrame.bodyData[j];
								
								if((bodyDataOther.bIsTracked != 0) && 
									(Mathf.Abs(bodyDataOther.position.z) < Mathf.Abs(bodyPos.z)))
								{
									bClosestUser = false;
									iClosestUserIndex = j;
									break;
								}
							}
						}
					}
					
					if(bClosestUser)
					{
                        // 向新用户列表添加第一个或最近的用户id
						if(!addedUsers.Contains(userId))
						{
							addedUsers.Add(userId);
							addedIndexes.Add(iClosestUserIndex);
						}
						
					}
				}

                // 将userId添加到新用户列表中
				if(!addedUsers.Contains(userId))
				{
					addedUsers.Add(userId);
					addedIndexes.Add(i);
				}

                // 将Kinect定位转换为世界定位
				bodyFrame.bodyData[i].position = bodyPos;
				//string debugText = String.Empty;

                // 处理特殊情况
				if(bodyData.joint[(int)KinectInterop.JointType.HipLeft].trackingState == KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.SpineBase].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.HipRight].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					bodyData.joint[(int)KinectInterop.JointType.HipLeft].trackingState = KinectInterop.TrackingState.Inferred;
					
					bodyData.joint[(int)KinectInterop.JointType.HipLeft].kinectPos = bodyData.joint[(int)KinectInterop.JointType.SpineBase].kinectPos +
						(bodyData.joint[(int)KinectInterop.JointType.SpineBase].kinectPos - bodyData.joint[(int)KinectInterop.JointType.HipRight].kinectPos);
					bodyData.joint[(int)KinectInterop.JointType.HipLeft].position = bodyData.joint[(int)KinectInterop.JointType.SpineBase].position +
						(bodyData.joint[(int)KinectInterop.JointType.SpineBase].position - bodyData.joint[(int)KinectInterop.JointType.HipRight].position);
					bodyData.joint[(int)KinectInterop.JointType.HipLeft].direction = bodyData.joint[(int)KinectInterop.JointType.HipLeft].position -
						bodyData.joint[(int)KinectInterop.JointType.SpineBase].position;
				}
				
				if(bodyData.joint[(int)KinectInterop.JointType.HipRight].trackingState == KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.SpineBase].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.HipLeft].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					bodyData.joint[(int)KinectInterop.JointType.HipRight].trackingState = KinectInterop.TrackingState.Inferred;
					
					bodyData.joint[(int)KinectInterop.JointType.HipRight].kinectPos = bodyData.joint[(int)KinectInterop.JointType.SpineBase].kinectPos +
						(bodyData.joint[(int)KinectInterop.JointType.SpineBase].kinectPos - bodyData.joint[(int)KinectInterop.JointType.HipLeft].kinectPos);
					bodyData.joint[(int)KinectInterop.JointType.HipRight].position = bodyData.joint[(int)KinectInterop.JointType.SpineBase].position +
						(bodyData.joint[(int)KinectInterop.JointType.SpineBase].position - bodyData.joint[(int)KinectInterop.JointType.HipLeft].position);
					bodyData.joint[(int)KinectInterop.JointType.HipRight].direction = bodyData.joint[(int)KinectInterop.JointType.HipRight].position -
						bodyData.joint[(int)KinectInterop.JointType.SpineBase].position;
				}
				
				if((bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].trackingState == KinectInterop.TrackingState.NotTracked &&
				    bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].trackingState != KinectInterop.TrackingState.NotTracked &&
				    bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].trackingState != KinectInterop.TrackingState.NotTracked))
				{
					bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].trackingState = KinectInterop.TrackingState.Inferred;
					
					bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].kinectPos = bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].kinectPos +
						(bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].kinectPos - bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].kinectPos);
					bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].position = bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].position +
						(bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].position - bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].position);
					bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].direction = bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].position -
						bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].position;
				}
				
				if((bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].trackingState == KinectInterop.TrackingState.NotTracked &&
				    bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].trackingState != KinectInterop.TrackingState.NotTracked &&
				    bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].trackingState != KinectInterop.TrackingState.NotTracked))
				{
					bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].trackingState = KinectInterop.TrackingState.Inferred;
					
					bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].kinectPos = bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].kinectPos +
						(bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].kinectPos - bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].kinectPos);
					bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].position = bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].position +
						(bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].position - bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].position);
					bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].direction = bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].position -
						bodyData.joint[(int)KinectInterop.JointType.SpineShoulder].position;
				}

				// calculate special directions
				if(bodyData.joint[(int)KinectInterop.JointType.HipLeft].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.HipRight].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					Vector3 posRHip = bodyData.joint[(int)KinectInterop.JointType.HipRight].position;
					Vector3 posLHip = bodyData.joint[(int)KinectInterop.JointType.HipLeft].position;
					
					bodyData.hipsDirection = posRHip - posLHip;
					bodyData.hipsDirection -= Vector3.Project(bodyData.hipsDirection, Vector3.up);
				}
				
				if(bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					Vector3 posRShoulder = bodyData.joint[(int)KinectInterop.JointType.ShoulderRight].position;
					Vector3 posLShoulder = bodyData.joint[(int)KinectInterop.JointType.ShoulderLeft].position;
					
					bodyData.shouldersDirection = posRShoulder - posLShoulder;
					bodyData.shouldersDirection -= Vector3.Project(bodyData.shouldersDirection, Vector3.up);
					
					Vector3 shouldersDir = bodyData.shouldersDirection;
					shouldersDir.z = -shouldersDir.z;
					
					Quaternion turnRot = Quaternion.FromToRotation(Vector3.right, shouldersDir);
					bodyData.bodyTurnAngle = turnRot.eulerAngles.y;
				}
				
				if(bodyData.joint[(int)KinectInterop.JointType.WristLeft].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.HandLeft].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					Vector3 pos1 = bodyData.joint[(int)KinectInterop.JointType.WristLeft].position;
					Vector3 pos2 = bodyData.joint[(int)KinectInterop.JointType.HandLeft].position;
					
					bodyData.leftHandDirection = pos2 - pos1;
				}
				
				if(bodyData.leftHandDirection != Vector3.zero &&
				   bodyData.joint[(int)KinectInterop.JointType.WristLeft].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.ThumbLeft].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					Vector3 pos1 = bodyData.joint[(int)KinectInterop.JointType.WristLeft].position;
					Vector3 pos2 = bodyData.joint[(int)KinectInterop.JointType.ThumbLeft].position;
					
					bodyData.leftThumbDirection = pos2 - pos1;
					bodyData.leftThumbDirection -= Vector3.Project(bodyData.leftThumbDirection, bodyData.leftHandDirection);

//					Vector3 shouldersDir = bodyData.shouldersDirection;
//					Vector3 thumbFwdDir = Vector3.forward;
//					Vector3.OrthoNormalize(ref shouldersDir, ref thumbFwdDir);
//
//					bodyData.leftThumbForward = thumbFwdDir;
//					bodyData.leftThumbForward -= Vector3.Project(bodyData.leftThumbForward, bodyData.leftHandDirection);
//					
//					bodyData.leftThumbAngle = Vector3.Angle(bodyData.leftThumbForward, bodyData.leftThumbDirection);
					bodyData.leftThumbAngle = bodyData.bodyTurnAngle;
				}
				
				if(bodyData.joint[(int)KinectInterop.JointType.WristRight].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.HandRight].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					Vector3 pos1 = bodyData.joint[(int)KinectInterop.JointType.WristRight].position;
					Vector3 pos2 = bodyData.joint[(int)KinectInterop.JointType.HandRight].position;
					
					bodyData.rightHandDirection = pos2 - pos1;
				}
				
				if(bodyData.rightHandDirection != Vector3.zero &&
				   bodyData.joint[(int)KinectInterop.JointType.WristRight].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.ThumbRight].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					Vector3 pos1 = bodyData.joint[(int)KinectInterop.JointType.WristRight].position;
					Vector3 pos2 = bodyData.joint[(int)KinectInterop.JointType.ThumbRight].position;
					
					bodyData.rightThumbDirection = pos2 - pos1;
					bodyData.rightThumbDirection -= Vector3.Project(bodyData.rightThumbDirection, bodyData.rightHandDirection);

//					Vector3 shouldersDir = bodyData.shouldersDirection;
//					Vector3 thumbFwdDir = Vector3.forward;
//					Vector3.OrthoNormalize(ref shouldersDir, ref thumbFwdDir);
//					
//					bodyData.rightThumbForward = thumbFwdDir;
//					bodyData.rightThumbForward -= Vector3.Project(bodyData.rightThumbForward, bodyData.rightHandDirection);
//
//					bodyData.rightThumbAngle = Vector3.Angle(bodyData.rightThumbForward, bodyData.rightThumbDirection);
					bodyData.rightThumbAngle = bodyData.bodyTurnAngle;
				}
				
				if(bodyData.joint[(int)KinectInterop.JointType.KneeLeft].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.AnkleLeft].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.FootLeft].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					Vector3 vFootProjected = Vector3.Project(bodyData.joint[(int)KinectInterop.JointType.FootLeft].direction, bodyData.joint[(int)KinectInterop.JointType.AnkleLeft].direction);
					
					bodyData.joint[(int)KinectInterop.JointType.AnkleLeft].kinectPos += vFootProjected;
					bodyData.joint[(int)KinectInterop.JointType.AnkleLeft].position += vFootProjected;
					bodyData.joint[(int)KinectInterop.JointType.FootLeft].direction -= vFootProjected;
				}
				
				if(bodyData.joint[(int)KinectInterop.JointType.KneeRight].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.AnkleRight].trackingState != KinectInterop.TrackingState.NotTracked &&
				   bodyData.joint[(int)KinectInterop.JointType.FootRight].trackingState != KinectInterop.TrackingState.NotTracked)
				{
					Vector3 vFootProjected = Vector3.Project(bodyData.joint[(int)KinectInterop.JointType.FootRight].direction, bodyData.joint[(int)KinectInterop.JointType.AnkleRight].direction);
					
					bodyData.joint[(int)KinectInterop.JointType.AnkleRight].kinectPos += vFootProjected;
					bodyData.joint[(int)KinectInterop.JointType.AnkleRight].position += vFootProjected;
					bodyData.joint[(int)KinectInterop.JointType.FootRight].direction -= vFootProjected;
				}

                // 计算身体关节的世界方向
				CalculateJointOrients(ref bodyData);

				if(sensorData != null && sensorData.sensorInterface != null)
				{
                    // 关节位置和方向是否有传感器专用的固定
					sensorData.sensorInterface.FixJointOrientations(sensorData, ref bodyData);
				}

                // 滤光片方向约束
				if(useBoneOrientationConstraints && boneConstraintsFilter != null)
				{
					boneConstraintsFilter.Constrain(ref bodyData);
				}
				
				lostUsers.Remove(userId);
				bodyFrame.bodyData[i] = bodyData;
			}
		}

        // 删除丢失的用户(如果有的话)
		if(lostUsers.Count > 0)
		{
			foreach(Int64 userId in lostUsers)
			{
				RemoveUser(userId);
			}
			
			lostUsers.Clear();
		}

        // 校准最新发现的用户
		if(addedUsers.Count > 0)
		{
			for(int i = 0; i < addedUsers.Count; i++)
			{
				Int64 userId = addedUsers[i];
				int userIndex = addedIndexes[i];

				CalibrateUser(userId, userIndex);
			}
			
			addedUsers.Clear();
			addedIndexes.Clear();
		}
	}

    // 将UserId添加到用户列表
    void CalibrateUser(Int64 userId, int bodyIndex)
    {
		if(!alUserIds.Contains(userId))
		{
			if(CheckForCalibrationPose(userId, bodyIndex, playerCalibrationPose))
			{
				int uidIndex = alUserIds.Count;
				Debug.Log("Adding user " + uidIndex + ", ID: " + userId + ", Index: " + bodyIndex);
				
				alUserIds.Add(userId);
				dictUserIdToIndex[userId] = bodyIndex;
				
				if(liPrimaryUserId == 0)
				{
					liPrimaryUserId = userId;
					
					if(liPrimaryUserId != 0)
					{
						if(calibrationText != null && calibrationText.GetComponent<GUIText>().text != "")
						{
							calibrationText.GetComponent<GUIText>().text = "";
						}
					}
				}
				
				for(int i = 0; i < avatarControllers.Count; i++)
				{
					AvatarController avatar = avatarControllers[i];
					
					if(avatar && avatar.playerIndex == uidIndex)
					{
						avatar.SuccessfulCalibration(userId);
					}
				}

                // 添加手势来检测，如果有的话。
				foreach(KinectGestures.Gestures gesture in playerCommonGestures)
				{
					DetectGesture(userId, gesture);
				}

                // 通知手势监听器关于新用户
				foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
				{
					listener.UserDetected(userId, 0);
				}
				
				ResetFilters();
			}
		}
    }

    // 删除一个失去了用户标识
	void RemoveUser(Int64 userId)
	{
		int uidIndex = alUserIds.IndexOf(userId);
		Debug.Log("Removing user " + uidIndex + ", ID: " + userId);
		
		for(int i = 0; i < avatarControllers.Count; i++)
		{
			AvatarController avatar = avatarControllers[i];
			
			if(avatar && avatar.playerIndex >= uidIndex && avatar.playerIndex < alUserIds.Count)
			{
				avatar.ResetToInitialPosition();
			}
		}

        // 通知手势监听器用户丢失
		foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
		{
			listener.UserLost(userId, 0);
		}

        // 清除该用户的手势列表
		ClearGestures(userId);

        // 清除此用户的校准数据
		if(playerCalibrationData.ContainsKey(userId))
		{
			playerCalibrationData.Remove(userId);
		}

        // 清理数据字典中过时的校准数据
		List<Int64> alCalDataKeys = new List<Int64>(playerCalibrationData.Keys);

		foreach(Int64 calUserID in alCalDataKeys)
		{
			KinectGestures.GestureData gestureData = playerCalibrationData[calUserID];

			if((gestureData.timestamp + 60f) < Time.realtimeSinceStartup)
			{
				playerCalibrationData.Remove(calUserID);
			}
		}

		alCalDataKeys.Clear();

        // 从全局用户列表中删除。
        alUserIds.Remove(userId);
		dictUserIdToIndex.Remove(userId);
		
		if(liPrimaryUserId == userId)
		{
			if(alUserIds.Count > 0)
			{
				liPrimaryUserId = alUserIds[0];
			}
			else
			{
				liPrimaryUserId = 0;
			}
		}
		
		for(int i = 0; i < avatarControllers.Count; i++)
		{
			AvatarController avatar = avatarControllers[i];
			
			if(avatar && avatar.playerIndex >= uidIndex && avatar.playerIndex < alUserIds.Count)
			{
				avatar.SuccessfulCalibration(alUserIds[avatar.playerIndex]);
			}
		}
		
		if(liPrimaryUserId == 0)
		{
			Debug.Log("Waiting for users.");
			
			if(calibrationText != null)
			{
				calibrationText.GetComponent<GUIText>().text = "WAITING FOR USERS";
			}
		}
	}

    // 在给定的纹理中绘制骨架。
	private void DrawSkeleton(Texture2D aTexture, ref KinectInterop.BodyData bodyData)
	{
		int jointsCount = sensorData.jointCount;
		
		for(int i = 0; i < jointsCount; i++)
		{
			int parent = (int)sensorData.sensorInterface.GetParentJoint((KinectInterop.JointType)i);
			
			if(bodyData.joint[i].trackingState != KinectInterop.TrackingState.NotTracked && 
			   bodyData.joint[parent].trackingState != KinectInterop.TrackingState.NotTracked)
			{
				Vector2 posParent = KinectInterop.MapSpacePointToDepthCoords(sensorData, bodyData.joint[parent].kinectPos);
				Vector2 posJoint = KinectInterop.MapSpacePointToDepthCoords(sensorData, bodyData.joint[i].kinectPos);
				
				if(posParent != Vector2.zero && posJoint != Vector2.zero)
				{
					//Color lineColor = playerJointsTracked[i] && playerJointsTracked[parent] ? Color.red : Color.yellow;
					DrawLine(aTexture, (int)posParent.x, (int)posParent.y, (int)posJoint.x, (int)posJoint.y, Color.yellow);
				}
			}
		}
		
		//aTexture.Apply();
	}

    // 在纹理中画出一条线
	private void DrawLine(Texture2D a_Texture, int x1, int y1, int x2, int y2, Color a_Color)
	{
		int width = sensorData != null ? sensorData.depthImageWidth : 0;
		int height = sensorData != null ? sensorData.depthImageHeight : 0;
		
		int dy = y2 - y1;
		int dx = x2 - x1;
	 
		int stepy = 1;
		if (dy < 0) 
		{
			dy = -dy; 
			stepy = -1;
		}
		
		int stepx = 1;
		if (dx < 0) 
		{
			dx = -dx; 
			stepx = -1;
		}
		
		dy <<= 1;
		dx <<= 1;
	 
		if(x1 >= 0 && x1 < width && y1 >= 0 && y1 < height)
			for(int x = -1; x <= 1; x++)
				for(int y = -1; y <= 1; y++)
					a_Texture.SetPixel(x1 + x, y1 + y, a_Color);
		
		if (dx > dy) 
		{
			int fraction = dy - (dx >> 1);
			
			while (x1 != x2) 
			{
				if (fraction >= 0) 
				{
					y1 += stepy;
					fraction -= dx;
				}
				
				x1 += stepx;
				fraction += dy;
				
				if(x1 >= 0 && x1 < width && y1 >= 0 && y1 < height)
					for(int x = -1; x <= 1; x++)
						for(int y = -1; y <= 1; y++)
							a_Texture.SetPixel(x1 + x, y1 + y, a_Color);
			}
		}
		else 
		{
			int fraction = dx - (dy >> 1);
			
			while (y1 != y2) 
			{
				if (fraction >= 0) 
				{
					x1 += stepx;
					fraction -= dy;
				}
				
				y1 += stepy;
				fraction += dx;
				
				if(x1 >= 0 && x1 < width && y1 >= 0 && y1 < height)
					for(int x = -1; x <= 1; x++)
						for(int y = -1; y <= 1; y++)
							a_Texture.SetPixel(x1 + x, y1 + y, a_Color);
			}
		}
		
	}

    // 计算关节的关节方向
	private void CalculateJointOrients(ref KinectInterop.BodyData bodyData)
	{
		int jointCount = bodyData.joint.Length;

		for(int j = 0; j < jointCount; j++)
		{
			int joint = j;

			KinectInterop.JointData jointData = bodyData.joint[joint];
			bool bJointValid = ignoreInferredJoints ? jointData.trackingState == KinectInterop.TrackingState.Tracked : jointData.trackingState != KinectInterop.TrackingState.NotTracked;

			if(bJointValid)
			{
				int nextJoint = (int)sensorData.sensorInterface.GetNextJoint((KinectInterop.JointType)joint);
				if(nextJoint != joint && nextJoint >= 0 && nextJoint < sensorData.jointCount)
				{
					KinectInterop.JointData nextJointData = bodyData.joint[nextJoint];
					bool bNextJointValid = ignoreInferredJoints ? nextJointData.trackingState == KinectInterop.TrackingState.Tracked : nextJointData.trackingState != KinectInterop.TrackingState.NotTracked;
					
					if(bNextJointValid)
					{
						Vector3 baseDir = KinectInterop.JointBaseDir[nextJoint];
						Vector3 jointDir = nextJointData.direction;
						jointDir.z = -jointDir.z;
						
						if((joint == (int)KinectInterop.JointType.ShoulderLeft) || (joint == (int)KinectInterop.JointType.ElbowLeft) ||
						   (joint == (int)KinectInterop.JointType.WristLeft) || (joint == (int)KinectInterop.JointType.HandLeft))
						{
							float angle = -bodyData.leftThumbAngle;
							Vector3 axis = jointDir;
							Quaternion armTurnRotation = Quaternion.AngleAxis(angle, axis);
							
							jointData.normalRotation = armTurnRotation * Quaternion.FromToRotation(baseDir, jointDir);
						}
						else if((joint == (int)KinectInterop.JointType.ShoulderRight) || (joint == (int)KinectInterop.JointType.ElbowRight) ||
						        (joint == (int)KinectInterop.JointType.WristRight) || (joint == (int)KinectInterop.JointType.HandRight))
						{
							float angle = -bodyData.rightThumbAngle;
							Vector3 axis = jointDir;
							Quaternion armTurnRotation = Quaternion.AngleAxis(angle, axis);
							
							jointData.normalRotation = armTurnRotation * Quaternion.FromToRotation(baseDir, jointDir);
						}
						else
						{
							jointData.normalRotation = Quaternion.FromToRotation(baseDir, jointDir);
						}
						
						if((joint == (int)KinectInterop.JointType.SpineBase) || (joint == (int)KinectInterop.JointType.SpineMid) || 
						   (joint == (int)KinectInterop.JointType.SpineShoulder) || (joint == (int)KinectInterop.JointType.Neck) ||
						   (joint == (int)KinectInterop.JointType.HipLeft) || (joint == (int)KinectInterop.JointType.HipRight) ||
						   (joint == (int)KinectInterop.JointType.KneeLeft) || (joint == (int)KinectInterop.JointType.KneeRight) ||
						   (joint == (int)KinectInterop.JointType.AnkleLeft) || (joint == (int)KinectInterop.JointType.AnkleRight))
						{
							baseDir = Vector3.right;
							jointDir = bodyData.shouldersDirection;
							jointDir.z = -jointDir.z;
							
							jointData.normalRotation *= Quaternion.FromToRotation(baseDir, jointDir);
						}
						
//						jointDir.x = -jointDir.x;
//						//jointDir.y = -jointDir.y;
//						
//						baseDir.x = -baseDir.x;
//						//baseDir.y = -baseDir.y;
//						
//						jointData.mirroredRotation = Quaternion.FromToRotation(baseDir, jointDir);
						
						Vector3 mirroredAngles = jointData.normalRotation.eulerAngles;
						mirroredAngles.y = -mirroredAngles.y;
						mirroredAngles.z = -mirroredAngles.z;
						
						jointData.mirroredRotation = Quaternion.Euler(mirroredAngles);
					}
					
				}
				else
				{
					jointData.normalRotation = Quaternion.identity;
					jointData.mirroredRotation = Quaternion.identity;
				}
			}

			bodyData.joint[joint] = jointData;
			
			if(joint == (int)KinectInterop.JointType.SpineBase)
			{
				bodyData.normalRotation = jointData.normalRotation;
				bodyData.mirroredRotation = jointData.mirroredRotation;
			}
		}
	}

    // 估计所定义的手势的当前状态
	private void CheckForGestures(Int64 UserId)
	{
		if(!playerGesturesData.ContainsKey(UserId) || !gesturesTrackingAtTime.ContainsKey(UserId))
			return;

        // 检查动作
		if(Time.realtimeSinceStartup >= gesturesTrackingAtTime[UserId])
		{
            //找到关节位置并跟踪
			int iAllJointsCount = sensorData.jointCount;
			bool[] playerJointsTracked = new bool[iAllJointsCount];
			Vector3[] playerJointsPos = new Vector3[iAllJointsCount];
			
			int[] aiNeededJointIndexes = KinectGestures.GetNeededJointIndexes();
			int iNeededJointsCount = aiNeededJointIndexes.Length;
			
			for(int i = 0; i < iNeededJointsCount; i++)
			{
				int joint = aiNeededJointIndexes[i];
				
				if(joint >= 0 && IsJointTracked(UserId, joint))
				{
					playerJointsTracked[joint] = true;
					playerJointsPos[joint] = GetJointPosition(UserId, joint);
				}
			}

            // 检查动作
			List<KinectGestures.GestureData> gesturesData = playerGesturesData[UserId];
			
			int listGestureSize = gesturesData.Count;
			float timestampNow = Time.realtimeSinceStartup;
			
			for(int g = 0; g < listGestureSize; g++)
			{
				KinectGestures.GestureData gestureData = gesturesData[g];
				
				if((timestampNow >= gestureData.startTrackingAtTime) && 
					!IsConflictingGestureInProgress(gestureData, ref gesturesData))
				{
					KinectGestures.CheckForGesture(UserId, ref gestureData, Time.realtimeSinceStartup, 
						ref playerJointsPos, ref playerJointsTracked);
					gesturesData[g] = gestureData;

					if(gestureData.complete)
					{
						gesturesTrackingAtTime[UserId] = timestampNow + minTimeBetweenGestures;
					}
				}
			}
			
			playerGesturesData[UserId] = gesturesData;
		}
	}
	
	private bool IsConflictingGestureInProgress(KinectGestures.GestureData gestureData, ref List<KinectGestures.GestureData> gesturesData)
	{
		foreach(KinectGestures.Gestures gesture in gestureData.checkForGestures)
		{
			int index = GetGestureIndex(gesture, ref gesturesData);
			
			if(index >= 0)
			{
				if(gesturesData[index].progress > 0f)
					return true;
			}
		}
		
		return false;
	}

    // 返回列表中手势的索引，否则返回-1
	private int GetGestureIndex(KinectGestures.Gestures gesture, ref List<KinectGestures.GestureData> gesturesData)
	{
		int listSize = gesturesData.Count;
	
		for(int i = 0; i < listSize; i++)
		{
			if(gesturesData[i].gesture == gesture)
				return i;
		}
		
		return -1;
	}

    // 检查给定用户的校准姿势是否完整
	private bool CheckForCalibrationPose(Int64 UserId, int bodyIndex, KinectGestures.Gestures calibrationGesture)
	{
		if(calibrationGesture == KinectGestures.Gestures.None)
			return true;

		KinectGestures.GestureData gestureData = playerCalibrationData.ContainsKey(UserId) ? 
			playerCalibrationData[UserId] : new KinectGestures.GestureData();

        // 如果需要，初始化手势数据
		if(gestureData.userId != UserId)
		{
			gestureData.userId = UserId;
			gestureData.gesture = calibrationGesture;
			gestureData.state = 0;
			gestureData.timestamp = Time.realtimeSinceStartup;
			gestureData.joint = 0;
			gestureData.progress = 0f;
			gestureData.complete = false;
			gestureData.cancelled = false;
		}

        // 找到关节位置并跟踪
		int iAllJointsCount = sensorData.jointCount;
		bool[] playerJointsTracked = new bool[iAllJointsCount];
		Vector3[] playerJointsPos = new Vector3[iAllJointsCount];
		
		int[] aiNeededJointIndexes = KinectGestures.GetNeededJointIndexes();
		int iNeededJointsCount = aiNeededJointIndexes.Length;
		
		for(int i = 0; i < iNeededJointsCount; i++)
		{
			int joint = aiNeededJointIndexes[i];
			
			if(joint >= 0)
			{
				KinectInterop.JointData jointData = bodyFrame.bodyData[bodyIndex].joint[joint];
				
				playerJointsTracked[joint] = jointData.trackingState != KinectInterop.TrackingState.NotTracked;
				playerJointsPos[joint] = jointData.kinectPos;
			}
		}

        // 估计姿态进步
		KinectGestures.CheckForGesture(UserId, ref gestureData, Time.realtimeSinceStartup, 
			ref playerJointsPos, ref playerJointsTracked);
		playerCalibrationData[UserId] = gestureData;

        // 检查手势是否完整。
		if(gestureData.complete)
		{
			gestureData.userId = 0;
			playerCalibrationData[UserId] = gestureData;

			return true;
		}

		return false;
	}
	
}

