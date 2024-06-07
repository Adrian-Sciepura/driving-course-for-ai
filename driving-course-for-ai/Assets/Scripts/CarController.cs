using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class CarController : Agent
{
    private float horizontalInput, verticalInput;
    private float currentSteerAngle, currentbreakForce;
    private bool isBreaking;

    private new Rigidbody rigidbody;
    private MapController mapController;

    private float timeInFreeField = 0.0f;
    private float GenerationTime = 0.0f;
    private int HowManyFreeSpacesAreOccupied = 0;
    private bool IsAlreadyGenerated = false;

    private DriverLearningData driverData;

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
                    AddReward(driverData.availableParkingSpaceData.OnEnterReward);
                break;
            case "ParkingArea":
                AddReward(driverData.areaData.ParkingArea_OnEnterReward);
                break;
            case "NearParkingSpaceArea":
                AddReward(driverData.areaData.NearParkingSpaceArea_OnEnterReward);
                break;
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        float speed = rigidbody.velocity.magnitude / 10.0f;
        switch (collision.gameObject.tag)
        {
            case "Fence":
                AddReward(driverData.fenceData.OnCollision_BaseReward + driverData.fenceData.OnCollision_SpeedMultiplier * speed);
                EndEpisode();
                break;
            case "OccupiedParkingSpace":
                AddReward(driverData.occupiedParkingSpaceData.OnCollisionReward + driverData.occupiedParkingSpaceData.OnCollision_SpeedMultiplier * speed);
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
            if (timeInFreeField >= driverData.availableParkingSpaceData.OnStay_MinimumTimeToStop && speed < driverData.availableParkingSpaceData.OnStay_MaximumSpeedtoStop)
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
                
                AddReward((
                    driverData.availableParkingSpaceData.OnStay_StopBaseReward +
                    driverData.availableParkingSpaceData.OnStay_StopCoverageMultiplier * result +
                    driverData.availableParkingSpaceData.OnStay_StopSpeedMultiplier * speed) / (HowManyFreeSpacesAreOccupied * HowManyFreeSpacesAreOccupied));
                EndEpisode();
            }
            else
            {
                AddReward(driverData.availableParkingSpaceData.OnStay_MovingReward / (HowManyFreeSpacesAreOccupied * HowManyFreeSpacesAreOccupied));
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
                    AddReward(driverData.availableParkingSpaceData.OnExitReward);
                }
                
                break;
            case "ParkingArea":
                AddReward(driverData.areaData.ParkingArea_OnExit_BaseReward + driverData.areaData.ParkingArea_OnExit_SpeedMultiplier * speed);
                EndEpisode();
                break;
            case "NearParkingSpaceArea":
                AddReward(driverData.areaData.NearParkingSpaceArea_OnExitReward);
                break;
        }
    }

    private void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        UpdateWheels();

        GenerationTime += Time.deltaTime;

        AddReward(driverData.basicData.RewardPerDeltaTime);

        if (GenerationTime >= driverData.basicData.MaxGenerationTime)
        {
            GenerationTime = 0.0f;
            AddReward(driverData.basicData.ExceededMaxGenerationTimeReward);
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
        driverData = GameManager.instance.driverLearningData;
        RandomizePosition();
        RandomizeRotation();
        StopWheelsMovement();
        rigidbody.velocity = Vector3.zero;

        if(driverData.mapRandomizationData.RandomizeEveryEpisode)
        {
            mapController.Randomize();
        }
        else if(!IsAlreadyGenerated)
        {
            mapController.Randomize();
            IsAlreadyGenerated = true;
        }
    }

    private void RandomizePosition() => transform.position = new Vector3(UnityEngine.Random.Range(PointA.position.x, PointB.position.x), transform.position.y, UnityEngine.Random.Range(PointA.position.z, PointB.position.z));
    private void RandomizeRotation() => transform.rotation = Quaternion.Euler(0.0f, driverData.carRandomizationData.RandomizeRotation ? UnityEngine.Random.Range(driverData.carRandomizationData.MinAngle, driverData.carRandomizationData.MaxAngle) : 0.0f, 0.0f);

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
