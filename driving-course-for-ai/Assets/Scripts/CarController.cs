using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class CarController : Agent
{
    private float horizontalInput, verticalInput;
    private float currentSteerAngle, currentbreakForce;
    private bool isBreaking;

    // Settings
    [SerializeField] private float motorForce, breakForce, maxSteerAngle;

    // Wheel Colliders
    [SerializeField] private WheelCollider frontLeftWheelCollider, frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider, rearRightWheelCollider;

    // Wheels
    [SerializeField] private Transform frontLeftWheelTransform, frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform, rearRightWheelTransform;


    // Defaults
    [SerializeField] private GameObject defaultTransform;

    public override void OnEpisodeBegin()
    {
        transform.position = defaultTransform.transform.position;
        transform.rotation = defaultTransform.transform.rotation;
    }


    public override void Initialize()
    {
        Time.timeScale = 8f;
    }


    // Vector
    // 0 - Break
    // 1 - Horizontal
    // 2 - Vertical
    public override void OnActionReceived(ActionBuffers actions)
    {
        isBreaking = false;//Convert.ToBoolean(actions.ContinuousActions[0]);
        horizontalInput = actions.ContinuousActions[1];
        verticalInput = actions.ContinuousActions[2];
    }

    // Vector
    // 0 - Break
    // 1 - Horizontal
    // 2 - Vertical
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Convert.ToSingle(Input.GetKey(KeyCode.Space));
        continuousActions[1] = Input.GetAxis("Horizontal");
        continuousActions[2] = Input.GetAxis("Vertical");
    }



    // Tags
    // Area
    // AvailableSpace
    // UnavailableSpace
    private void OnTriggerEnter(Collider other)
    {        
        switch(other.gameObject.tag)
        {
            case "AvailableSpace":
            {
                AddReward(2f);
                break;
            }
            case "UnavailableSpace":
            {

                break;
            }
            case "Area":
            {
                AddReward(0.2f);
                break;
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        switch (collision.gameObject.tag)
        {
            case "UnavailableSpace":
            {
                    Debug.Log("dupa");
                AddReward(-2f);
                EndEpisode();
                break;
            }
        }

    }

    private void OnTriggerExit(Collider other)
    {
        switch (other.gameObject.tag)
        {
            case "AvailableSpace":
            {
                AddReward(-1.5f);
                break;
            }
            case "UnavailableSpace":
            {
                
                break;
            }
            case "Area":
            {
                AddReward(-1f);
                EndEpisode();
                break;
            }
        }
    }







    private void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        UpdateWheels();
    }

    private void GetInput()
    {
        // Steering Input

    }

    private void HandleMotor()
    {
        frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
        frontRightWheelCollider.motorTorque = verticalInput * motorForce;
        currentbreakForce = isBreaking ? breakForce : 0f;
        ApplyBreaking();
    }

    private void ApplyBreaking()
    {
        frontRightWheelCollider.brakeTorque = currentbreakForce;
        frontLeftWheelCollider.brakeTorque = currentbreakForce;
        rearLeftWheelCollider.brakeTorque = currentbreakForce;
        rearRightWheelCollider.brakeTorque = currentbreakForce;
    }

    private void HandleSteering()
    {
        currentSteerAngle = maxSteerAngle * horizontalInput;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }

    private void UpdateWheels()
    {
        UpdateSingleWheel(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateSingleWheel(frontRightWheelCollider, frontRightWheelTransform);
        UpdateSingleWheel(rearRightWheelCollider, rearRightWheelTransform);
        UpdateSingleWheel(rearLeftWheelCollider, rearLeftWheelTransform);
    }

    private void UpdateSingleWheel(WheelCollider wheelCollider, Transform wheelTransform)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        wheelTransform.rotation = rot;
        wheelTransform.position = pos;
    }
}