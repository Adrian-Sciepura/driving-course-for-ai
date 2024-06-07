using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEditor;
using UnityEngine;

public class CarController : Agent
{
    private float horizontalInput, verticalInput;
    private float currentSteerAngle, currentbreakForce;
    private bool isBreaking;

    private Rigidbody rigidbody;
    private MapController mapController;

    private float timeInFreeField = 0.0f;
    private float MaxGenerationTime;
    private float GenerationTime = 0.0f;
    private int HowManyFreeSpacesAreOccupied = 0;

    public Transform rayOrigin;
    public float rayLength = 15.0f;

    private Transform nearestParkingSpace;
    private float lastDistanceToParkingSpace = float.MaxValue;

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

    private bool IsAlreadyGenerated = false;

    public override void OnEpisodeBegin()
    {
        CreateNewSetup();
        //FindNearestParkingSpace();
    }

    public override void Initialize()
    {
        MaxGenerationTime = 120.0f;
        GenerationTime = 0.0f;
        mapController = GetComponent<MapController>();
        rigidbody = GetComponent<Rigidbody>();
    }

    // Vector
    // 0 - Horizontal
    // 1 - VerticalF
    public override void OnActionReceived(ActionBuffers actions)
    {
        horizontalInput = actions.ContinuousActions[0];
        verticalInput = actions.ContinuousActions[1];
        isBreaking = actions.ContinuousActions[2] == 1.0f;
    }

    // Vector
    // 0 - Horizontal
    // 1 - Vertical
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
        continuousActions[2] = Input.GetKey(KeyCode.Space) ? 1.0f : 0.0f;
    }

    private void OnTriggerEnter(Collider other)
    {
        switch (other.gameObject.tag)
        {
            case "AvailableParkingSpace":
                HowManyFreeSpacesAreOccupied++;

                if(HowManyFreeSpacesAreOccupied == 1)
                {
                    AddReward(4.0f);
                }
                break;
            case "ParkingArea":
                AddReward(2.0f);
                break;
            case "NearParkingSpaceArea":
                AddReward(0.3f);
                break;
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        float speed = rigidbody.velocity.magnitude / 10.0f;
        switch (collision.gameObject.tag)
        {
            case "Fence":
                AddReward(-2.5f - 2.0f * speed);
                EndEpisode();
                break;
            case "OccupiedParkingSpace":
                AddReward(-1.5f - 1.2f * speed);
                EndEpisode();
                break;
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.gameObject.tag == "AvailableParkingSpace")
        {
            timeInFreeField += Time.deltaTime;
            float speed = rigidbody.velocity.magnitude / 10.0f;
            if (timeInFreeField >= 2.0f && speed < 0.1)
            {
                var carBounds = CarCollider.bounds;
                var otherBounds = other.bounds;
                int containedPoints = 0;

                for (int i = 0; i < 500; i++)
                {
                    Vector3 randomPoint = GetRandomPointInBounds(carBounds);
                    if (otherBounds.Contains(randomPoint))
                        containedPoints++;
                }

                float result = containedPoints / 500.0f;
                
                AddReward((2.0f + 6.0f * result - 3.0f * speed) / (HowManyFreeSpacesAreOccupied * HowManyFreeSpacesAreOccupied));
                EndEpisode();
            }
            else
            {
                AddReward(0.005f / (HowManyFreeSpacesAreOccupied * HowManyFreeSpacesAreOccupied));
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        float speed = rigidbody.velocity.magnitude / 10.0f;

        switch (other.gameObject.tag)
        {
            case "AvailableParkingSpace":
                HowManyFreeSpacesAreOccupied--;

                if (HowManyFreeSpacesAreOccupied == 0)
                {
                    timeInFreeField = 0.0f;
                    AddReward(-4.0f);
                }
                
                break;
            case "ParkingArea":
                AddReward(-2.0f - 1.5f * speed);
                EndEpisode();
                break;
            case "NearParkingSpaceArea":
                AddReward(-0.3f);
                break;
        }
    }

    private void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        UpdateWheels();

        GenerationTime += Time.deltaTime;

        AddReward(-0.005f);

        if (GenerationTime >= MaxGenerationTime)
        {
            GenerationTime = 0.0f;
            AddReward(-3.0f);
            EndEpisode();
        }
    }

    private void FindNearestParkingSpace()
    {
        // Find all available parking spaces
        List<GameObject> availableParkingSpaces = mapController.FreeParkingFields;

        // Find the nearest parking space
        float closestDistance = float.MaxValue;
        foreach (GameObject parkingSpace in availableParkingSpaces)
        {
            float distance = Vector3.Distance(transform.position, parkingSpace.transform.position);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                nearestParkingSpace = parkingSpace.transform;
            }
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
        StopWheelsMovement();
        rigidbody.velocity = Vector3.zero;
        mapController.Randomize();


        //FindNearestParkingSpace();
        //lastDistanceToParkingSpace = Vector3.Distance(transform.position, nearestParkingSpace.position);
    }

    private void RandomizePosition() => transform.position = new Vector3(UnityEngine.Random.Range(PointA.position.x, PointB.position.x), transform.position.y, UnityEngine.Random.Range(PointA.position.z, PointB.position.z));
    private void RandomizeRotation() => transform.rotation = Quaternion.Euler(0.0f, UnityEngine.Random.Range(-60.0f, 60.0f), 0.0f);

    private void StopWheelsMovement()
    {
        frontLeftWheelCollider.motorTorque = 0f;
        frontRightWheelCollider.motorTorque = 0f;
        frontRightWheelCollider.brakeTorque = 0f;
        frontLeftWheelCollider.brakeTorque = 0f;
        rearLeftWheelCollider.brakeTorque = 0f;
        rearRightWheelCollider.brakeTorque = 0f;

        currentSteerAngle = 0f;
        frontLeftWheelCollider.steerAngle = 0f;
        frontRightWheelCollider.steerAngle = 0f;
        UpdateWheels();
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
