using UnityEngine;

public class MapController : MonoBehaviour
{
    [SerializeField] private Transform[] ParkFields;
    [SerializeField] private int MaxFreeFields = 6;

    [SerializeField] private Transform SpawnRoot;
    [SerializeField] private GameObject ParkedCarPrefab;
    [SerializeField] private GameObject FreeSpacePrefab;

    private GameObject[] spawnedObjects;

    private void Awake()
    {
        spawnedObjects = new GameObject[ParkFields.Length];
    }

    public void Randomize()
    {
        int allFields = ParkFields.Length;
        
        for (int i = 0; i < allFields; i++)
        {
            if (spawnedObjects[i] != null)
                Destroy(spawnedObjects[i]);
            spawnedObjects[i] = null;
        }

        
        int freeFields = Random.Range(1, MaxFreeFields);

        while (freeFields > 0)
        {
            int pos = UnityEngine.Random.Range(0, allFields - 1);
            if (spawnedObjects[pos] == null)
            {
                spawnedObjects[pos] = Instantiate(FreeSpacePrefab, ParkFields[pos].position, ParkFields[pos].rotation);
                spawnedObjects[pos].transform.parent = SpawnRoot;
                freeFields--;
            }
        }

        for(int i = 0; i < allFields; i++)
        {
            if (spawnedObjects[i] == null)
            {
                spawnedObjects[i] = Instantiate(ParkedCarPrefab, ParkFields[i].position, ParkFields[i].rotation);
                
                Vector3 pos = spawnedObjects[i].transform.position;
                pos.y = 0.01f;

                spawnedObjects[i].transform.parent = SpawnRoot;
                spawnedObjects[i].transform.position = pos;
                spawnedObjects[i].transform.rotation = Quaternion.Euler(0.0f, 90.0f + FreeSpacePrefab.transform.rotation.y, 0.0f);
            }
        }
    }
}
