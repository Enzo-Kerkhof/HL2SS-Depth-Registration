using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Timer : MonoBehaviour
{
    public float timeRemaining = 10;
    public GameObject progress;
    public bool timerIsRunning = false;
    private void Start()
    {
        // Starts the timer automatically
        
    }
    void Update()
    {
        if (timerIsRunning)
        {
            progress.SetActive(true);
            if (timeRemaining > 0)
            {
                timeRemaining -= Time.deltaTime;
            }
            else
            {
                Debug.Log("Time has run out!");
                timeRemaining = 0;
                timerIsRunning = false;
                progress.SetActive(false);
            }
        }
    }
    public void On_Speech()
    {
        timerIsRunning = true;
    }
}