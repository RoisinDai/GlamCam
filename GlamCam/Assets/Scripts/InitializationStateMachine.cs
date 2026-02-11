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
    public GameObject handCursorLeft;

    [Header("Measurement Source")]
    public MultiSourceManager multiSourceManager;

    [Header("Timing")]
    public float transitionSeconds = 2f;

    private State _currentState = State.Init;
    public State CurrentState => _currentState;
    
    private Coroutine _transitionCoroutine;

    // Cached references to Main UI pieces we want to toggle
    private GameObject _uiRig;

    private GameObject _topLeftArrow, _bottomLeftArrow, _dressLeftArrow, _fullbodyLeftArrow, _hatLeftArrow, _accessoriesLeftArrow;
    private GameObject _topImage, _bottomImage, _dressImage, _fullbodyImage, _hatImage, _accessoriesImage;

    private GameObject _pillBg;
    private GameObject _divider;
    private GameObject _clearAllBtn;
    private GameObject _resetBtn;
    private GameObject _viewMannequinBtn;
    private GameObject _viewSceneBtn;
    
    private void Start()
    {
        // MainRig should always be active for this approach
        if (mainRig != null) mainRig.SetActive(true);

        // Auto-find MultiSourceManager if not assigned
        if (multiSourceManager == null)
        {
            multiSourceManager = FindObjectOfType<MultiSourceManager>();
            if (multiSourceManager == null)
                Debug.LogWarning("[InitializationStateMachine] MultiSourceManager not found.");
        }

        CacheMainUiReferences();
        EnterInitState();
    }

    private void Update()
    {
        if (_currentState != State.Init) return;
        if (multiSourceManager == null) return;

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

        // MainRig stays ON, but we hide the Main phase UI pieces
        SetMainUiVisible(false);

        SetActive(handCursor, false);
        SetActive(handCursorLeft, false);
    }

    private void EnterTransitionState()
    {
        _currentState = State.Transition;

        SetActive(initRig, false);
        SetActive(transitionRig, true);

        // Still keep Main UI pieces hidden during transition
        SetMainUiVisible(false);

        SetActive(handCursor, false);
        SetActive(handCursorLeft, false);
    }

    private void EnterMainState()
    {
        _currentState = State.Main;

        SetActive(initRig, false);
        SetActive(transitionRig, false);

        // Show Main phase UI pieces now
        SetMainUiVisible(true);

        SetActive(handCursor, true);
        SetActive(handCursorLeft, true);
    }

    // Public API
    public void ResetToInit()
    {
        // Stop any in-flight transition
        if (_transitionCoroutine != null)
        {
            StopCoroutine(_transitionCoroutine);
            _transitionCoroutine = null;
        }

        // Clear all clothing on the avatar
        ClothingRulesCoordinator.ClearAllClothing();

        EnterInitState();
    }

    private void SetMainUiVisible(bool visible)
    {
        // Toggle the LeftArrow + Image objects inside each selector
        SetActive(_topLeftArrow, visible);
        SetActive(_bottomLeftArrow, visible);
        SetActive(_dressLeftArrow, visible);
        SetActive(_fullbodyLeftArrow, visible);
        SetActive(_hatLeftArrow, visible);
        SetActive(_accessoriesLeftArrow, visible);

        SetActive(_topImage, visible);
        SetActive(_bottomImage, visible);
        SetActive(_dressImage, visible);
        SetActive(_fullbodyImage, visible);
        SetActive(_hatImage, visible);
        SetActive(_accessoriesImage, visible);

        // PillNav children: PillBG, Divider
        SetActive(_pillBg, visible);
        SetActive(_divider, visible);

        // Buttons
        SetActive(_clearAllBtn, visible);
        SetActive(_resetBtn, visible);
    }

    private void CacheMainUiReferences()
    {
        if (mainRig == null) return;

        // MainRig/UIRig
        _uiRig = FindGo(mainRig, "UIRig");

        // Selectors
        // Each selector: LeftArrow (always hidden) + XImage (toggled)
        _topLeftArrow = FindGo(mainRig, "UIRig/TopSelector/LeftArrow");
        _topImage = FindGo(mainRig, "UIRig/TopSelector/TopImage");

        _bottomLeftArrow = FindGo(mainRig, "UIRig/BottomSelector/LeftArrow");
        _bottomImage = FindGo(mainRig, "UIRig/BottomSelector/BottomImage");

        _dressLeftArrow = FindGo(mainRig, "UIRig/DressSelector/LeftArrow");
        _dressImage = FindGo(mainRig, "UIRig/DressSelector/DressImage");

        _fullbodyLeftArrow = FindGo(mainRig, "UIRig/FullbodySelector/LeftArrow");
        _fullbodyImage = FindGo(mainRig, "UIRig/FullbodySelector/FullbodyImage");

        _hatLeftArrow = FindGo(mainRig, "UIRig/HatSelector/LeftArrow");
        _hatImage = FindGo(mainRig, "UIRig/HatSelector/HatImage");

        _accessoriesLeftArrow = FindGo(mainRig, "UIRig/AccessoriesSelector/LeftArrow");
        _accessoriesImage = FindGo(mainRig, "UIRig/AccessoriesSelector/AccessoryImage");

        // PillNav children: PillBG, Divider
        _pillBg = FindGo(mainRig, "PillNav/PillBG");
        _divider = FindGo(mainRig, "PillNav/Divider");

        // Buttons
        _clearAllBtn = FindGo(mainRig, "ClearAllBtn");
        _resetBtn = FindGo(mainRig, "ResetBtn");
    }

    private GameObject FindGo(GameObject root, string path)
    {
        if (root == null) return null;
        var t = root.transform.Find(path);
        if (t == null)
        {
            Debug.LogWarning($"[InitializationStateMachine] Missing path under '{root.name}': {path}");
            return null;
        }
        return t.gameObject;
    }

    private void SetActive(GameObject go, bool active)
    {
        if (go != null) go.SetActive(active);
    }
}