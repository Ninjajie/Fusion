using UnityEngine;

[AddComponentMenu("Camera-Control/Key Mouse Orbit")]
public class KeyboardOrbit : MonoBehaviour
{
    public Transform target;
    public float distanceMin = 10.0f;
    public float distanceMax = 15.0f;
    public float distanceInitial = 12.5f;
    public float scrollSpeed = 1.0f;

    public float xSpeed = 250.0f;
    public float ySpeed = 120.0f;

    public int yMinLimit = -20;
    public int yMaxLimit = 80;

    private float x = 0.0f;
    private float y = 0.0f;
    private float distanceCurrent = 0.0f;

    void Start()
    {
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;

        distanceCurrent = distanceInitial;

        // Make the rigid body not change rotation
        if (GetComponent<Rigidbody>())
        {
            GetComponent<Rigidbody>().freezeRotation = true;
        }
    }

    private void LateUpdate()
    {
        if (target)
        {
            x += Input.GetAxis("Horizontal") * xSpeed * 0.02f;
            y -= Input.GetAxis("Vertical") * ySpeed * 0.02f;
            distanceCurrent -= Input.GetAxis("Mouse ScrollWheel") * scrollSpeed;

            distanceCurrent = Mathf.Clamp(distanceCurrent, distanceMin, distanceMax);
            y = ClampAngle(y, yMinLimit, yMaxLimit);

            Quaternion rotation = Quaternion.Euler(y, x, 0);
            Vector3 position = rotation * new Vector3(0.0f, 0.0f, -distanceCurrent) + target.position;

            transform.rotation = rotation;
            transform.position = position;
        }
    }

    private static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360)
        {
            angle += 360;
        }

        if (angle > 360)
        {
            angle -= 360;
        }

        return Mathf.Clamp(angle, min, max);
    }
}