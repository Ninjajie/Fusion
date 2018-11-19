using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class InitialRigidbodyVelocity : MonoBehaviour
{
    public float speed = 10.0f;
    
    void Start()
    {
        GetComponent<Rigidbody>().velocity = transform.TransformDirection(Vector3.forward) * speed;
    }
}