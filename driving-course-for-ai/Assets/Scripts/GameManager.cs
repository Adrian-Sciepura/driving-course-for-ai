using System.IO;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public static GameManager instance;
    private DriverLearningData _driverLearningData;
    
    public DriverLearningData driverLearningData
    {
        get
        {
            if (_driverLearningData == null)
                LoadDriverDataFromFile();

            return _driverLearningData;
        }
    }

    private void Awake()
    {
        if(instance == null)
        {
            instance = this;
        }
        else
        {
            Destroy(this);
            return;
        }

        DontDestroyOnLoad(this);
    }

    private void Update()
    {
        if(Input.GetKeyDown(KeyCode.R))
            LoadDriverDataFromFile();
    }

    private void LoadDriverDataFromFile()
    {
        if(File.Exists("AI_LearningData.json"))
        {
            using (FileStream fileStream = File.OpenRead("AI_LearningData.json"))
                _driverLearningData = UnityEngine.JsonUtility.FromJson<DriverLearningData>(new StreamReader(fileStream).ReadToEnd());
        }
        else
        {
            _driverLearningData = DriverLearningData.CreateDefault();
            File.WriteAllText("AI_LearningData.json", JsonUtility.ToJson(driverLearningData, true));
        }
    }
}
