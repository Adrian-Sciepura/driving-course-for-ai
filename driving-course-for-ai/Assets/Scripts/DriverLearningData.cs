using System;

[Serializable]
public class DriverLearningData
{
    [Serializable]
    public class MapRandomizationData
    {
        public bool RandomizeEveryEpisode;
        public int MinFreeSpaces;
    }

    [Serializable]
    public class CarRandomizationData
    {
        public bool RandomizeRotation;
        public float MinAngle;
        public float MaxAngle;
    }

    [Serializable]
    public class BasicData
    {
        public float MaxGenerationTime;
        public float ExceededMaxGenerationTimeReward;
        public float RewardPerDeltaTime;
    }

    [Serializable]
    public class AvailableParkingSpaceData
    {
        public float OnEnterReward;
        public float OnExitReward;

        public float OnStay_MinimumTimeToStop;
        public float OnStay_MaximumSpeedtoStop;

        public float OnStay_StopBaseReward;
        public float OnStay_StopCoverageMultiplier;
        public float OnStay_StopSpeedMultiplier;

        public float OnStay_MovingReward;
    }

    [Serializable]
    public class OccupiedParkingSpaceData
    {
        public float OnCollisionReward;
        public float OnCollision_SpeedMultiplier;
    }

    [Serializable]
    public class  AreaData
    {
        public float ParkingArea_OnEnterReward;
        public float ParkingArea_OnExit_BaseReward;
        public float ParkingArea_OnExit_SpeedMultiplier;

        public float NearParkingSpaceArea_OnEnterReward;
        public float NearParkingSpaceArea_OnExitReward;
    }

    [Serializable]
    public class FenceData
    {
        public float OnCollision_BaseReward;
        public float OnCollision_SpeedMultiplier;
    }

    public MapRandomizationData mapRandomizationData;
    public CarRandomizationData carRandomizationData;
    public BasicData basicData;
    public AvailableParkingSpaceData availableParkingSpaceData;
    public OccupiedParkingSpaceData occupiedParkingSpaceData;
    public AreaData areaData;
    public FenceData fenceData;


    public static DriverLearningData CreateDefault()
    {
        var driverData = new DriverLearningData()
        {
            mapRandomizationData = new MapRandomizationData()
            {
                RandomizeEveryEpisode = true,
                MinFreeSpaces = 3
            },

            carRandomizationData = new CarRandomizationData()
            {
                RandomizeRotation = true,
                MinAngle = -60.0f,
                MaxAngle = 60.0f
            },

            basicData = new BasicData()
            {
                MaxGenerationTime = 120.0f,
                ExceededMaxGenerationTimeReward = -3.0f,
                RewardPerDeltaTime = -0.005f
            },

            availableParkingSpaceData = new AvailableParkingSpaceData()
            {
                OnEnterReward = 4.0f,
                OnExitReward = -4.0f,

                OnStay_MinimumTimeToStop = 2.0f,
                OnStay_MaximumSpeedtoStop = 0.1f,

                OnStay_StopBaseReward = 2.0f,
                OnStay_StopCoverageMultiplier = 6.0f,
                OnStay_StopSpeedMultiplier = -3.0f,

                OnStay_MovingReward = 0.005f
            },

            occupiedParkingSpaceData = new OccupiedParkingSpaceData()
            {
                OnCollisionReward = -1.5f,
                OnCollision_SpeedMultiplier = -1.2f
            },

            areaData = new AreaData()
            {
                ParkingArea_OnEnterReward = 2.0f,
                ParkingArea_OnExit_BaseReward = -2.0f,
                ParkingArea_OnExit_SpeedMultiplier = -1.5f,

                NearParkingSpaceArea_OnEnterReward = 0.3f,
                NearParkingSpaceArea_OnExitReward = -0.3f
            },

            fenceData = new FenceData()
            {
                OnCollision_BaseReward = -2.5f,
                OnCollision_SpeedMultiplier = -2.0f
            }
        };

        return driverData;
    }
}