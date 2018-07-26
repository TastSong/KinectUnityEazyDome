using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PresentationScript : MonoBehaviour 
{
	public bool slideChangeWithGestures = true;
	public bool slideChangeWithKeys = true;
	public float spinSpeed = 5;
	
	public bool autoChangeAlfterDelay = false;
	public float slideChangeAfterDelay = 10;
	
	public List<Texture> slideTextures;
	public List<GameObject> horizontalSides;

    // �����ʾ��ά���ݼ�λ���û��ĺ���(true)���û���ǰ��(false)
	public bool isBehindUser = false;
	
	private int maxSides = 0;
	private int maxTextures = 0;
	private int side = 0;
	private int tex = 0;
	private bool isSpinning = false;
	private float slideWaitUntil;
	private Quaternion targetRotation;
	
	private GestureListener gestureListener;
	

	
	void Start() 
	{
        // ���������
		Cursor.visible = false;

        // �������õ�Ƭ������
		maxSides = horizontalSides.Count;
		maxTextures = slideTextures.Count;

        // �ӳٵ�һ�Żõ�Ƭ
		slideWaitUntil = Time.realtimeSinceStartup + slideChangeAfterDelay;
		
		targetRotation = transform.rotation;
		isSpinning = false;
		
		tex = 0;
		side = 0;
		
		if(horizontalSides[side] && horizontalSides[side].GetComponent<Renderer>())
		{
			horizontalSides[side].GetComponent<Renderer>().material.mainTexture = slideTextures[tex];
		}

        //�Ѷ���������
		gestureListener = Camera.main.GetComponent<GestureListener>();
	}
	
	void Update() 
	{
        //���û���û�����Ҫ����Update()
		KinectManager kinectManager = KinectManager.Instance;
		if(autoChangeAlfterDelay && (!kinectManager || !kinectManager.IsInitialized() || !kinectManager.IsUserDetected()))
			return;
		
		if(!isSpinning)
		{
			if(slideChangeWithKeys)
			{
				if(Input.GetKeyDown(KeyCode.PageDown))
					RotateToNext();
				else if(Input.GetKeyDown(KeyCode.PageUp))
					RotateToPrevious();
			}
			
			if(slideChangeWithGestures && gestureListener)
			{
				if(gestureListener.IsSwipeLeft())
					RotateToNext();
				else if(gestureListener.IsSwipeRight())
					RotateToPrevious();
			}

            // ����ڸ������ӳ�ʱ����Ƿ��Զ��л�����
			if(autoChangeAlfterDelay && Time.realtimeSinceStartup >= slideWaitUntil)
			{
				RotateToNext();
			}
		}
		else
		{
            // ������ʾ
			transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, spinSpeed * Time.deltaTime);

            // ���ת���Ƿ�ﵽĿ����ת������ǣ�ֹͣ��ת��
			float deltaTargetX = Mathf.Abs(targetRotation.eulerAngles.x - transform.rotation.eulerAngles.x);
			float deltaTargetY = Mathf.Abs(targetRotation.eulerAngles.y - transform.rotation.eulerAngles.y);
			
			if(deltaTargetX < 1f && deltaTargetY < 1f)
			{
				// delay the slide
				slideWaitUntil = Time.realtimeSinceStartup + slideChangeAfterDelay;
				isSpinning = false;
			}
		}
	}
	
	
	private void RotateToNext()
	{
        // ������һ������õ�Ƭ
		tex = (tex + 1) % maxTextures;
		
		if(!isBehindUser)
		{
			side = (side + 1) % maxSides;
		}
		else
		{
			if(side <= 0)
				side = maxSides - 1;
			else
				side -= 1;
		}

		if(horizontalSides[side] && horizontalSides[side].GetComponent<Renderer>())
		{
			horizontalSides[side].GetComponent<Renderer>().material.mainTexture = slideTextures[tex];
		}

        // ��ת��ʾ
		float yawRotation = !isBehindUser ? 360f / maxSides : -360f / maxSides;
		Vector3 rotateDegrees = new Vector3(0f, yawRotation, 0f);
		targetRotation *= Quaternion.Euler(rotateDegrees);
		isSpinning = true;
	}
	
	
	private void RotateToPrevious()
	{
        //����֮ǰ������õ�Ƭ
		if(tex <= 0)
			tex = maxTextures - 1;
		else
			tex -= 1;
		
		if(!isBehindUser)
		{
			if(side <= 0)
				side = maxSides - 1;
			else
				side -= 1;
		}
		else
		{
			side = (side + 1) % maxSides;
		}
		
		if(horizontalSides[side] && horizontalSides[side].GetComponent<Renderer>())
		{
			horizontalSides[side].GetComponent<Renderer>().material.mainTexture = slideTextures[tex];
		}

        // ��ת��ʾ
		float yawRotation = !isBehindUser ? -360f / maxSides : 360f / maxSides;
		Vector3 rotateDegrees = new Vector3(0f, yawRotation, 0f);
		targetRotation *= Quaternion.Euler(rotateDegrees);
		isSpinning = true;
	}
	
	
}
