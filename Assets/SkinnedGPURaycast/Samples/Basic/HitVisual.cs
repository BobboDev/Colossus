using UnityEngine;

public class HitVisual : MonoBehaviour
{
    public float lifetime = 1f;

    private float birthTime = 0f;

    // Start is called before the first frame update
    void Start()
    {
        birthTime = Time.time;
        transform.localScale = Vector3.one * .05f;
    }

    // Update is called once per frame
    void Update()
    {
        if(Time.time > (birthTime + lifetime))
        {
            Destroy(gameObject);
        }
    }
}
