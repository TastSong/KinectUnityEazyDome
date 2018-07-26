using UnityEngine;
//using Windows.Kinect;

using System;
using System.Collections;
using System.Collections.Generic;

public class KinectManager : MonoBehaviour 
{
    // ����������ж��(����Ϊ��λ)��
	public float sensorHeight = 1.0f;

    // Kinect����(�Ƕ�)
	public float sensorAngle = 0f;

    // ����Bool�����Ƿ���պͼ����û�ӳ�䡣
	public bool computeUserMap = false;

    // ����Bool�����Ƿ���պͼ����ɫ��ͼ��
	public bool computeColorMap = false;

    // ȷ������Bool�Ƿ���պͼ�������ͼ
	public bool computeInfraredMap = false;

    // ����Bool�����Ƿ���GUI����ʾ�û�ӳ�䡣
	public bool displayUserMap = false;

    // ȷ���Ƿ���GUI����ʾ��ɫӳ��
	public bool displayColorMap = false;

    // ȷ���Ƿ����û���ͼ����ʾ�Ǽ���
	public bool displaySkeletonLines = false;
	
	//��������������ָ����ȺͲ�ɫ��ͼ��ʹ�õ�ͼ���ȣ��������ȵ�%���߶��Ǹ��ݿ��������ġ�
    // ����ٷֱ�Ϊ�㣬�����ڲ����㣬��ƥ�����ͼ���ѡ����Ⱥ͸߶�
	public float DisplayMapsWidthPercent = 20f;

    // ����Bool�����Ƿ�ʹ�ö�Դ�Ķ���������еĻ�
	public bool useMultiSourceReader = false;

    // ��С���û����룬�Դ���Ǽ�����
	public float minUserDistance = 0.5f;

    // ����û����룬����еĻ���0��ʾû������������
	public float maxUserDistance = 0f;

    // �ж��Ƿ�ֻ���������û�
	public bool detectClosestUser = true;

    // ����Bool�����Ƿ�ֻʹ�ø��ٵĽڵ�(�����ƶϵĽڵ�)
	public bool ignoreInferredJoints = true;

    // ƽ��������ѡ��
	public enum Smoothing : int { None, Default, Medium, Aggressive }
	public Smoothing smoothing = Smoothing.Default;

    // ����Boolȷ��ʹ�ø��ӹ�����
	public bool useBoneOrientationConstraints = true;
	//public bool useBoneOrientationsFilter = false;

    // ����Kinect�û����Ƶ�avatarcontroller�����б�
	public List<AvatarController> avatarControllers;

    // �����Ҫ����ÿ����Ա����У����
	public KinectGestures.Gestures playerCalibrationPose;

    // ÿ�������Ҫ���������б�
	public List<KinectGestures.Gestures> playerCommonGestures;

    // ���Ƽ��֮������ʱ��
	public float minTimeBetweenGestures = 0.7f;

    // �����������б����Ǳ���ʵ��KinectGestures.GestureListenerInterface
	public List<MonoBehaviour> gestureListeners;

    // ��ʾ��Ϣ��GUI�ı���
	public GUIText calibrationText;


    // Bool�����Ƿ�Kinect�Ѿ���ʼ��
	private bool kinectInitialized = false;

    // KinectManager�ĵ���ʵ��
	private static KinectManager instance = null;

    // ���õĴ������ӿ�
	private List<DepthSensorInterface> sensorInterfaces = null;
    // ��ҪSensorData�ṹ
	private KinectInterop.SensorData sensorData = null;

    // ��Ⱥ��û���ͼ
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

    // Kinect����֡����
	private KinectInterop.BodyFrameData bodyFrame;
	//private Int64 lastBodyFrameTime = 0;

    // �����û��б�
	private List<Int64> alUserIds;
	private Dictionary<Int64, int> dictUserIdToIndex;
	
	// Primary (first or closest) user ID
	private Int64 liPrimaryUserId = 0;

    // Kinect�������
	private Matrix4x4 kinectToWorld = Matrix4x4.zero;
	//private Matrix4x4 mOrient = Matrix4x4.zero;

    // Ϊÿ�����У׼��������
	private Dictionary<Int64, KinectGestures.GestureData> playerCalibrationData = new Dictionary<Int64, KinectGestures.GestureData>();

    // ��̬���ݺͲ���
	private Dictionary<Int64, List<KinectGestures.GestureData>> playerGesturesData = new Dictionary<Int64, List<KinectGestures.GestureData>>();
	private Dictionary<Int64, float> gesturesTrackingAtTime = new Dictionary<Int64, float>();

    //�����������б����Ǳ���ʵ��KinectGestures.GestureListenerInterface
	public List<KinectGestures.GestureListenerInterface> gestureListenerInts;

    // ���������ʵ��
	private JointPositionsFilter jointPositionFilter = null;
	private BoneOrientationsConstraint boneConstraintsFilter = null;
	//private BoneOrientationsFilter boneOrientationFilter = null;

    // ���ص���KinectManagerʵ��
    public static KinectManager Instance
    {
        get
        {
            return instance;
        }
    }

    // ���Kinect�Ƿ��ʼ����׼��ʹ�á����û�У�����kine -sensor��ʼ����������һ������
	public static bool IsKinectInitialized()
	{
		return instance != null ? instance.kinectInitialized : false;
	}

    // ���Kinect�Ƿ��ʼ����׼��ʹ�á����û�У�����kine -sensor��ʼ����������һ������
	public bool IsInitialized()
	{
		return kinectInitialized;
	}

    // �����ɴ��������ٵ�ʬ������
	public int GetSensorBodyCount()
	{
		return sensorData != null ? sensorData.bodyCount : 0;
	}

    // ���ش��������ٵĹؽ�����
	public int GetSensorJointCount()
	{
		return sensorData != null ? sensorData.jointCount : 0;
	}

    // ���عؽ������и����ؽڵ�����
	public int GetJointIndex(KinectInterop.JointType joint)
	{
		if(sensorData != null && sensorData.sensorInterface != null)
		{
			return sensorData.sensorInterface.GetJointIndex(joint);
		}
		
		// fallback - index matches the joint
		return (int)joint;
	}

    // ���ظ����������Ĺؽ�
	public KinectInterop.JointType GetJointAtIndex(int index)
	{
		if(sensorData != null && sensorData.sensorInterface != null)
		{
			return sensorData.sensorInterface.GetJointAtIndex(index);
		}
		
		// fallback - index matches the joint
		return (KinectInterop.JointType)index;
	}

    // ���ظ����ؽڵĸ��ؽ�
	public KinectInterop.JointType GetParentJoint(KinectInterop.JointType joint)
	{
		if(sensorData != null && sensorData.sensorInterface != null)
		{
			return sensorData.sensorInterface.GetParentJoint(joint);
		}

        // ����-������ͬ�Ĺؽ�(��ĩ�˹ؽ�)
		return joint;
	}

    // ���ش�����֧�ֵĲ�ɫͼ��Ŀ�ȡ�
	public int GetColorImageWidth()
	{
		return sensorData != null ? sensorData.colorImageWidth : 0;
	}

    // ���ش�����֧�ֵĲ�ɫͼ��ĸ߶�
	public int GetColorImageHeight()
	{
		return sensorData != null ? sensorData.colorImageHeight : 0;
	}

    // ���ش�����֧�ֵ����ͼ��Ŀ��
	public int GetDepthImageWidth()
	{
		return sensorData != null ? sensorData.depthImageWidth : 0;
	}

    // ���ش�����֧�ֵ����ͼ��ĸ߶�
	public int GetDepthImageHeight()
	{
		return sensorData != null ? sensorData.depthImageHeight : 0;
	}

    // ���ComputeUserMapΪ�棬�򷵻�ԭʼ���/�û�����
	public ushort[] GetRawDepthMap()
	{
		return sensorData != null ? sensorData.depthImage : null;
	}

    // ����ԭʼ�������ݣ����ComputeInfraredMapΪ��
	public ushort[] GetRawInfraredMap()
	{
		return sensorData != null ? sensorData.infraredImage : null;
	}


    // ���ComputeUserMapΪ�棬�������ͼ��/�û�ֱ��ͼ����
    public Texture2D GetUsersLblTex()
    { 
		return usersLblTex;
	}

    // ������ɫͼ���������ComputeColorMapΪ��
	public Texture2D GetUsersClrTex()
	{ 
		return usersClrTex;
	}

	// �����������ǰ��⵽����һ���û����򷵻�true
	public bool IsUserDetected()
	{
		return kinectInitialized && (alUserIds.Count > 0);
	}

    // ����û���У׼��׼��ʹ�ã��򷵻�true
	public bool IsUserCalibrated(Int64 userId)
	{
		return dictUserIdToIndex.ContainsKey(userId);
	}

    // ���ص�ǰ��⵽���û�����
	public int GetUsersCount()
	{
		return alUserIds.Count;
	}

    // ͨ����������������UserID
	public Int64 GetUserIdByIndex(int i)
	{
		if(i >= 0 && i < alUserIds.Count)
		{
			return alUserIds[i];
		}
		
		return 0;
	}

    //�������û���UserID(��һ����������û�)������еĻ�
	public Int64 GetPrimaryUserID()
	{
		return liPrimaryUserId;
	}

    // �����µ����û�ID���Ը��Ļ�û���
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

    // �����û��������ݣ������ڵ���Ŀ��
    // ��ֱ�Ӹı�ṹ�е�������
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

    // �����û�λ�ã�����ڶ���������������Ϊ��λ
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

    // ��������ڶ������������û���ת
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

    // ����ָ���û������ؽڵ�ԭʼ����״̬
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

    // ������ڸ���ָ���û��ĸ����ؽڣ��򷵻�true
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

    // ����ָ���û�������λ�ã�����ڴ�����������Ϊ��λ��
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

    // ����ָ���û�������λ�ã�����ڴ�����������Ϊ��λ��
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

    // ����ָ���û��Ĺؽڷ�������ڸ��ؽ�
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

    // ����ָ���û���firstJoint��secondJoint֮��ķ���
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

    // ����ָ���û�����ڶ����������Ĺؽ���ת
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

    // ����ָ���û�����ڶ����������Ĺؽ���ת
    // �������ֵ�ߣ��򷵻�true;�������ֵ�ͣ��򷵻�false;���û���ҵ��û����򷵻�false
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

    // ����û����������Ŷ��Ƿ��
    // �������ֵ�ߣ��򷵻�true;�������ֵ�ͣ��򷵻�false;���û���ҵ��û����򷵻�false
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

    // �����û�������״̬
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

    // �����û�������״̬
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

    // ����ָ���û������Ľ���������Ϊ��λ��
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

    // ����ָ���û����ֵĽ����򣬵�λΪ��
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

    // ���ComputeUserMapΪ�棬�򷵻��ض����ص��������
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

    // ����3D���Vector2�����ӳ�����ꡣ���Kinectû�г�ʼ������Ϊ0
	public Vector2 MapSpacePointToDepthCoords(Vector3 posPoint)
	{
		Vector2 posDepth = Vector2.zero;
		
		if(kinectInitialized)
		{
			posDepth = KinectInterop.MapSpacePointToDepthCoords(sensorData, posPoint);
		}
		
		return posDepth;
	}

    // ���ظ�����ȵ����ɫӳ������
	public Vector2 MapDepthPointToColorCoords(Vector2 posPoint, ushort depthValue)
	{
		Vector2 posColor = Vector3.zero;
		
		if(kinectInitialized)
		{
			posColor = KinectInterop.MapDepthPointToColorCoords(sensorData, posPoint, depthValue);
		}
		
		return posColor;
	}

    // ɾ����ǰ��⵽��kinect�û������������µļ��/У׼����
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

    // �������ݹ�����
	public void ResetFilters()
	{
		if(jointPositionFilter != null)
		{
			jointPositionFilter.Reset();
		}
	}

    // ��ָ���û���⵽�������б������һ������
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

    // Ϊָ���û��ĸ�������������������״̬
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

    // Ϊָ���û������м�⵽��������������gesture-data״̬��
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

    // ��ָ���û���⵽�������б���ɾ������������
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

    // ���ָ���û���⵽�������б�
	public void ClearGestures(Int64 UserId)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			gesturesData.Clear();
			playerGesturesData[UserId] = gesturesData;
		}
	}

    // ����ָ���û���⵽�������б��м�⵽�����Ƶļ���
	public int GetGesturesCount(Int64 UserId)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;

		if(gesturesData != null)
		{
			return gesturesData.Count;
		}

		return 0;
	}

    // ����ָ���û���⵽�����Ƶ��б�
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

    // ���������������ָ���û���⵽�������б��У��򷵻�true
	public bool IsGestureDetected(Int64 UserId, KinectGestures.Gestures gesture)
	{
		List<KinectGestures.GestureData> gesturesData = playerGesturesData.ContainsKey(UserId) ? playerGesturesData[UserId] : null;
		int index = gesturesData != null ? GetGestureIndex(gesture, ref gesturesData) : -1;

		return index >= 0;
	}

    // ���ָ���û��ĸ�����������ɣ��򷵻�true
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

    // ���ָ���û��ĸ������Ʊ�ȡ�����򷵻�true
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

    // ����ָ���û��ĸ������Ƶķ�Χ[0,1]�Ľ�չ
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

    // Ϊָ�����û����ظ������Ƶĵ�ǰ����Ļλ�á�
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
            // ��ʼ�����õĴ������ӿ�
			bool bNeedRestart = false;
			sensorInterfaces = KinectInterop.InitSensorInterfaces(ref bNeedRestart);

			if(bNeedRestart)
			{
                // ���¼�����ͬ��ˮƽ
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
            // ���Գ�ʼ��Ĭ�ϵ�Kinect2������
			KinectInterop.FrameSource dwFlags = KinectInterop.FrameSource.TypeBody;
			if(computeUserMap)
				dwFlags |= KinectInterop.FrameSource.TypeDepth | KinectInterop.FrameSource.TypeBodyIndex;
			if(computeColorMap)
				dwFlags |= KinectInterop.FrameSource.TypeColor;
			if(computeInfraredMap)
				dwFlags |= KinectInterop.FrameSource.TypeInfrared;

            // ��Ĭ�ϵĴ�����
			sensorData = KinectInterop.OpenDefaultSensor(sensorInterfaces, dwFlags, sensorAngle, useMultiSourceReader);
			if (sensorData == null)
			{
				throw new Exception("OpenDefaultSensor failed");
			}

            //����ת������- kinect������
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

        // ���õ���ʵ��
		instance = this;

        // init�Ǽܽṹ
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

        // ��ʼ�����ݹ�����
		jointPositionFilter = new JointPositionsFilter();
		jointPositionFilter.Init(smoothParameters);

        // ��ʼ����ȡ��Լ��
		if(useBoneOrientationConstraints)
		{
			boneConstraintsFilter = new BoneOrientationsConstraint();
			boneConstraintsFilter.AddDefaultConstraints();
			//boneConstraintsFilter.SetDebugText(calibrationText);
		}

        // ��ȡ���������
		Rect cameraRect = Camera.main.pixelRect;

        // �����Ҫ�����ٷֱȼ����ͼ��Ⱥ͸߶�
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

            // ��ʼ����Ⱥͱ�ǩӳ����صĶ���
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

            // ��ʼ����ɫ��ͼ��صĶ���
			usersClrTex = new Texture2D(sensorData.colorImageWidth, sensorData.colorImageHeight, TextureFormat.RGBA32, false);
			usersClrRect = new Rect(cameraRect.width - displayWidth, cameraRect.height, displayWidth, -displayHeight);
			usersClrSize = sensorData.colorImageWidth * sensorData.colorImageHeight;
			
//			if(computeUserMap && displayColorMap)
//			{
//				usersMapRect.x -= cameraRect.width * displayMapsWidthPercent;
//			}
		}

        // �������ֳ��Զ��ҵ����õĻ��������
		if(avatarControllers.Count == 0)
		{
			AvatarController[] avatars = FindObjectsOfType(typeof(AvatarController)) as AvatarController[];
			
			foreach(AvatarController avatar in avatars)
			{
				avatarControllers.Add(avatar);
			}
		}

        // ��ʼ���û��б��԰��������û���
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
        // �˳�ʱ�ر�Kinect��
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

                // ���˸��ٵĹؽ�λ��
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

                // ������1������
				CheckForGestures(userId);

                // �������������
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

    // ���²�ɫͼ��
	void UpdateColorMap()
	{
		//usersClrTex.SetPixels32(colorImage.pixels);
		usersClrTex.LoadRawTextureData(sensorData.colorImage);
		usersClrTex.Apply();
	}

    // �����û�ֱ��ͼ
    void UpdateUserMap()
    {
		//if(KinectInterop.PollUserHistogramFrame(ref userHistogramImage, computeColorMap))
		{
            // ���û�ֱ��ͼ
			//usersLblTex.SetPixels32(userHistogramImage.pixels);

			UpdateUserHistogramImage();
			usersLblTex.SetPixels32(usersHistogramImage);

            // ��������
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

    // �����û������ͼ��
	void UpdateInfraredMap()
	{
        // ����ʲôҲ����
	}

    //�����û�ֱ��ͼ��ͼ
	void UpdateUserHistogramImage()
	{
		int numOfPoints = 0;
		Array.Clear(usersHistogramMap, 0, usersHistogramMap.Length);

        //������ȵ��ۻ�ֱ��ͼ
		for (int i = 0; i < usersMapSize; i++)
		{
            // ֻ��������û������
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

        // ���ݱ�ǩ��ͼ�����ֱ��ͼ����ʵ�ʵ��û�����
		Color32 clrClear = Color.clear;
		for (int i = 0; i < usersMapSize; i++)
		{
			ushort userMap = sensorData.bodyIndexImage[i];
			ushort userDepth = sensorData.depthImage[i];

			if(userDepth > 5000)
				userDepth = 5000;
			
			ushort nowUserPixel = userMap != 255 ? (ushort)((userMap << 13) | userDepth) : userDepth;
			ushort wasUserPixel = usersPrevState[i];

            // ֻ�����Ѹ��ĵ�����
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

    // ����Ǽ����ݽ��д���
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
                //������λ��
				Vector3 bodyPos = bodyData.position;
				
				if(liPrimaryUserId == 0)
				{
                    // ������Ƿ���������û�
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
                        // �����û��б���ӵ�һ����������û�id
						if(!addedUsers.Contains(userId))
						{
							addedUsers.Add(userId);
							addedIndexes.Add(iClosestUserIndex);
						}
						
					}
				}

                // ��userId��ӵ����û��б���
				if(!addedUsers.Contains(userId))
				{
					addedUsers.Add(userId);
					addedIndexes.Add(i);
				}

                // ��Kinect��λת��Ϊ���綨λ
				bodyFrame.bodyData[i].position = bodyPos;
				//string debugText = String.Empty;

                // �����������
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

                // ��������ؽڵ����緽��
				CalculateJointOrients(ref bodyData);

				if(sensorData != null && sensorData.sensorInterface != null)
				{
                    // �ؽ�λ�úͷ����Ƿ��д�����ר�õĹ̶�
					sensorData.sensorInterface.FixJointOrientations(sensorData, ref bodyData);
				}

                // �˹�Ƭ����Լ��
				if(useBoneOrientationConstraints && boneConstraintsFilter != null)
				{
					boneConstraintsFilter.Constrain(ref bodyData);
				}
				
				lostUsers.Remove(userId);
				bodyFrame.bodyData[i] = bodyData;
			}
		}

        // ɾ����ʧ���û�(����еĻ�)
		if(lostUsers.Count > 0)
		{
			foreach(Int64 userId in lostUsers)
			{
				RemoveUser(userId);
			}
			
			lostUsers.Clear();
		}

        // У׼���·��ֵ��û�
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

    // ��UserId��ӵ��û��б�
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

                // �����������⣬����еĻ���
				foreach(KinectGestures.Gestures gesture in playerCommonGestures)
				{
					DetectGesture(userId, gesture);
				}

                // ֪ͨ���Ƽ������������û�
				foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
				{
					listener.UserDetected(userId, 0);
				}
				
				ResetFilters();
			}
		}
    }

    // ɾ��һ��ʧȥ���û���ʶ
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

        // ֪ͨ���Ƽ������û���ʧ
		foreach(KinectGestures.GestureListenerInterface listener in gestureListeners)
		{
			listener.UserLost(userId, 0);
		}

        // ������û��������б�
		ClearGestures(userId);

        // ������û���У׼����
		if(playerCalibrationData.ContainsKey(userId))
		{
			playerCalibrationData.Remove(userId);
		}

        // ���������ֵ��й�ʱ��У׼����
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

        // ��ȫ���û��б���ɾ����
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

    // �ڸ����������л��ƹǼܡ�
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

    // �������л���һ����
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

    // ����ؽڵĹؽڷ���
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

    // ��������������Ƶĵ�ǰ״̬
	private void CheckForGestures(Int64 UserId)
	{
		if(!playerGesturesData.ContainsKey(UserId) || !gesturesTrackingAtTime.ContainsKey(UserId))
			return;

        // ��鶯��
		if(Time.realtimeSinceStartup >= gesturesTrackingAtTime[UserId])
		{
            //�ҵ��ؽ�λ�ò�����
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

            // ��鶯��
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

    // �����б������Ƶ����������򷵻�-1
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

    // �������û���У׼�����Ƿ�����
	private bool CheckForCalibrationPose(Int64 UserId, int bodyIndex, KinectGestures.Gestures calibrationGesture)
	{
		if(calibrationGesture == KinectGestures.Gestures.None)
			return true;

		KinectGestures.GestureData gestureData = playerCalibrationData.ContainsKey(UserId) ? 
			playerCalibrationData[UserId] : new KinectGestures.GestureData();

        // �����Ҫ����ʼ����������
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

        // �ҵ��ؽ�λ�ò�����
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

        // ������̬����
		KinectGestures.CheckForGesture(UserId, ref gestureData, Time.realtimeSinceStartup, 
			ref playerJointsPos, ref playerJointsTracked);
		playerCalibrationData[UserId] = gestureData;

        // ��������Ƿ�������
		if(gestureData.complete)
		{
			gestureData.userId = 0;
			playerCalibrationData[UserId] = gestureData;

			return true;
		}

		return false;
	}
	
}

