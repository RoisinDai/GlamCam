using System.Collections;
using UnityEngine;

public class InitializationStateMachine : MonoBehaviour
{
    public enum State
    {
        Init,
        Transition,
        Main
    }

    [Header("Rigs (Scene References)")]
    public GameObject initRig;
    public GameObject transitionRig;
    public GameObject mainRig;
    public GameObject handCursor;

    [Header("Measurement Source")]
    public MultiSourceManager multiSourceManager;

    [Header("Timing")]
    public float transitionSeconds = 2f;

    private State _currentState = State.Init;
    private Coroutine _transitionCoroutine;

    private void Start()
    {
        // Safety: auto-find MultiSourceManager if not assigned
        if (multiSourceManager == null)
        {
            multiSourceManager = FindObjectOfType<MultiSourceManager>();
            if (multiSourceManager == null)
                Debug.LogWarning("[InitializationStateMachine] MultiSourceManager not found.");
        }

        EnterInitState();
    }

    private void Update()
    {
        if (_currentState != State.Init) return;
        if (multiSourceManager == null) return;

        // Poll measurement readiness
        if (multiSourceManager.IsMeasured)
        {
            BeginTransition();
        }
    }

    private void BeginTransition()
    {
        if (_transitionCoroutine != null) return;

        _transitionCoroutine = StartCoroutine(TransitionRoutine());
    }

    private IEnumerator TransitionRoutine()
    {
        EnterTransitionState();

        yield return new WaitForSeconds(transitionSeconds);

        EnterMainState();

        _transitionCoroutine = null;
    }

    private void EnterInitState()
    {
        _currentState = State.Init;

        SetActive(initRig, true);
        SetActive(transitionRig, false);
        SetActive(mainRig, false);
        SetActive(handCursor, false);
    }

    private void EnterTransitionState()
    {
        _currentState = State.Transition;

        SetActive(initRig, false);
        SetActive(transitionRig, true);
        SetActive(mainRig, false);
        SetActive(handCursor, false);
    }

    private void EnterMainState()
    {
        _currentState = State.Main;

        SetActive(initRig, false);
        SetActive(transitionRig, false);
        SetActive(mainRig, true);
        SetActive(handCursor, true);
    }

    // public api
    public void ResetToInit()
    {
        // Stop any in-flight transition
        if (_transitionCoroutine != null)
        {
            StopCoroutine(_transitionCoroutine);
            _transitionCoroutine = null;
        }

        EnterInitState();
    }

    // helper
    private void SetActive(GameObject go, bool active)
    {
        if (go != null)
            go.SetActive(active);
    }
}
