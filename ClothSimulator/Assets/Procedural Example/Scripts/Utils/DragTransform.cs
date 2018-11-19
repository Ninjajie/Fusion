using System.Collections;
using UnityEngine;

public class DragTransform : MonoBehaviour
{
    public Color mouseOverColor = Color.blue;
    private Color originalColor;

    void Start()
    {
        originalColor = GetComponent<Renderer>().sharedMaterial.color;
    }

    private void OnMouseEnter()
    {
        GetComponent<Renderer>().material.color = mouseOverColor;
    }

    private void OnMouseExit()
    {
        GetComponent<Renderer>().material.color = originalColor;
    }

    private IEnumerator OnMouseDown()
    {
        Vector3 screenSpace = Camera.main.WorldToScreenPoint(transform.position);
        Vector3 offset = transform.position -
                         Camera.main.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y,
                             screenSpace.z));
        while (Input.GetMouseButton(0))
        {
            Vector3 curScreenSpace = new Vector3(Input.mousePosition.x, Input.mousePosition.y, screenSpace.z);
            Vector3 curPosition = Camera.main.ScreenToWorldPoint(curScreenSpace) + offset;
            transform.position = curPosition;
            yield return null;
        }
    }
}