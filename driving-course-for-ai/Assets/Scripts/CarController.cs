using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class CarController : Agent
{
    private float horizontalInput, verticalInput;
    private float currentSteerAngle, currentbreakForce;
    private bool isBreaking;

    private MapController mapController;
    private int fieldCount = 0;
    private float timeInFreeField = 0.0f;

    private float MaxGenerationTime;
    private float GenerationTime = 0.0f;

    // Settings
    [SerializeField] private float motorForce, breakForce, maxSteerAngle;

    // Wheel Colliders
    [SerializeField] private WheelCollider frontLeftWheelCollider, frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider, rearRightWheelCollider;

    // Wheels
    [SerializeField] private Transform frontLeftWheelTransform, frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform, rearRightWheelTransform;

    // SpawnArea
    [SerializeField] private Transform PointA;
    [SerializeField] private Transform PointB;

    [SerializeField] private Collider CarCollider;

    public override void OnEpisodeBegin()
    {
        CreateNewSetup();
    }


    public override void Initialize()
    {
        Time.timeScale = 3.0f;
        MaxGenerationTime = 30.0f;
        GenerationTime = 0.0f;
        mapController = GetComponent<MapController>();
        CreateNewSetup();
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
                AddReward(0.5f);
                break;
            }
            case "Area":
            {
                fieldCount++;
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
                AddReward(-2f);
                break;
            }
        }

    }

    private void OnTriggerStay(Collider other)
    {
        switch(other.gameObject.tag)
        {
            case "AvailableSpace":
            {
                timeInFreeField += Time.deltaTime;
                if (timeInFreeField >= 2.0f)
                {
                    var carBounds = CarCollider.bounds;
                    var otherBounds = other.bounds;
                    int containedPoints = 0;

                    for(int i = 0; i < 500; i++)
                    {
                        Vector3 randomPoint = GetRandomPointInBounds(carBounds);
                        if (otherBounds.Contains(randomPoint))
                            containedPoints++;
                    }

                    float result = containedPoints / 500.0f;

                    AddReward(3.0f * result);
                    EndEpisode();
                }
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
                AddReward(-0.6f);
                timeInFreeField = 0.0f;
                break;
            }
            case "Area":
            {
                fieldCount--;
                if (fieldCount == 0)
                {
                    AddReward(-2f);
                    EndEpisode();
                }
                break;
            }
        }
    }

    private void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        UpdateWheels();

        GenerationTime += Time.deltaTime;
        
        if (GenerationTime >= MaxGenerationTime)
        {
            GenerationTime = 0.0f;
            AddReward(-1f);
            EndEpisode();
        }
    }

    Vector3 GetRandomPointInBounds(Bounds bounds)
    {
        return new Vector3(
            UnityEngine.Random.Range(bounds.min.x, bounds.max.x),
            UnityEngine.Random.Range(bounds.min.y, bounds.max.y),
            UnityEngine.Random.Range(bounds.min.z, bounds.max.z)
        );
    }

    private void CreateNewSetup()
    {
        RandomizePosition();
        RandomizeRotation();
        mapController.Randomize();
    }
    private void RandomizePosition() => transform.position = new Vector3(UnityEngine.Random.Range(PointA.position.x, PointB.position.x), transform.position.y, UnityEngine.Random.Range(PointA.position.z, PointB.position.z));
    private void RandomizeRotation() => transform.rotation = Quaternion.Euler(0.0f, UnityEngine.Random.Range(-50.0f, 50.0f), 0.0f);

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