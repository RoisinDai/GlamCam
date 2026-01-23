const { useState, useEffect, useRef, useCallback } = React;

const DWELL_TIME = 800; // ms

const CLOSET_CATEGORIES = window.CLOSET_CATEGORIES || [];
const CATEGORY_TYPE = window.CATEGORY_TYPE || {};
const CLOTHING_NAMES = window.CLOTHING_NAMES || {};

// Closet drawer “pages”
function buildCategoryItems(categoryKey) {
  const names = CLOTHING_NAMES[categoryKey] || [];
  const type = CATEGORY_TYPE[categoryKey] || "unknown";

  return names.map((name) => ({
    type,
    name,
    image: `/static/${name}.png`,
    label: name,
  }));
}

const CLOTHING_PAGES = Object.fromEntries(
  Object.keys(CLOTHING_NAMES).map((categoryKey) => [
    categoryKey,
    buildCategoryItems(categoryKey),
  ])
);

// Layout constants for "attached" tab + drawer
const DRAWER_TOP = "5%";
const DRAWER_HEIGHT = "75%";
const DRAWER_WIDTH = 180;

const TAB_WIDTH = 54;
const TAB_GAP = 30;
const TAB_RIGHT_MARGIN_CLOSED = 12;
const TAB_HEIGHT_RATIO = 0.8;

// Drawer list layout
const DRAWER_GAP = 16;
const DRAWER_PADDING_X = 10; // matches drawer padding "20px 10px"
const VISIBLE_ITEMS = 5;
const ITEM_SIZE = DRAWER_WIDTH - 2 * DRAWER_PADDING_X; // 160
const SCROLL_VIEW_HEIGHT =
  VISIBLE_ITEMS * ITEM_SIZE + (VISIBLE_ITEMS - 1) * DRAWER_GAP;

// Custom scrollbar sizing (LEFT)
const SCROLL_TRACK_WIDTH = 18;
const SCROLL_TRACK_INSET = 0; // 0 = flush with drawer edge
const SCROLL_CONTENT_PAD_LEFT = SCROLL_TRACK_WIDTH + 8;

// Auto-close timer
const AUTO_CLOSE_MS = 2000;

// Header sizing constants
const DRAWER_LOGO_HEIGHT = 50;
const DRAWER_CLEAR_HEIGHT = 56;
const DRAWER_HEADER_GAP = 10;
const DRAWER_PAGE_LABEL_HEIGHT = 34;

const DRAWER_HEADER_HEIGHT =
  DRAWER_LOGO_HEIGHT +
  DRAWER_HEADER_GAP +
  DRAWER_CLEAR_HEIGHT +
  DRAWER_HEADER_GAP +
  DRAWER_PAGE_LABEL_HEIGHT;

// VideoFeed component
function VideoFeed() {
  return React.createElement("img", {
    id: "video-feed",
    src: "/video_feed",
    alt: "Live Fashion Feed",
    style: {
      position: "absolute",
      top: 0,
      left: 0,
      width: "100%",
      height: "100%",
      objectFit: "cover",
      zIndex: 1,
      transform: "scaleX(1)",
    },
  });
}

// ClosetTab component: each segment is its own hover target
const ClosetTab = React.forwardRef(
  ({ isOpen, activeIndex, pressedIndex, segmentRefs }, ref) => {
    const right = isOpen
      ? `${DRAWER_WIDTH + TAB_GAP}px`
      : `${TAB_RIGHT_MARGIN_CLOSED}px`;

    return React.createElement(
      "div",
      {
        ref,
        id: "closet-tab",
        style: {
          position: "absolute",
          top: DRAWER_TOP,
          right,
          width: `${TAB_WIDTH}px`,
          height: `calc(${DRAWER_HEIGHT} * ${TAB_HEIGHT_RATIO})`,
          zIndex: 3,
          backgroundColor: "rgba(255, 255, 255, 0.25)",
          borderRadius: "999px",
          backdropFilter: "blur(8px)",
          display: "flex",
          flexDirection: "column",
          justifyContent: "stretch",
          alignItems: "stretch",
          padding: 0,
          overflow: "hidden",
          transition: "right 0.4s ease-in-out",
        },
      },
      CLOSET_CATEGORIES.map((cat, index) => {
        const isPressed = pressedIndex !== null && pressedIndex === index;
        const isActive = activeIndex !== null && activeIndex === index;

        return React.createElement(
          "div",
          {
            key: cat.key,
            ref: (el) => {
              if (el) segmentRefs.current[index] = el;
            },
            className: "closet-tab-segment",
            "data-seg-index": String(index),
            style: {
              flex: 1,
              width: "100%",
              position: "relative",
              display: "flex",
              alignItems: "center",
              justifyContent: "center",
              backgroundColor: isActive
                ? "#7685c5"
                : isPressed
                ? "rgba(255, 255, 255, 0.38)"
                : "transparent",
              transition: "background-color 0.12s ease",
            },
          },
          index === 0
            ? null
            : React.createElement("div", {
                style: {
                  position: "absolute",
                  top: 0,
                  left: 0,
                  right: 0,
                  height: "1px",
                  backgroundColor: "rgba(255, 255, 255, 0.6)",
                },
              }),
          React.createElement("img", {
            src: cat.img,
            alt: cat.alt,
            draggable: false,
            style: {
              width: cat.key === "Tops" || cat.key === "Hats" ? "95%" : "110%",
              height: cat.key === "Tops" || cat.key === "Hats" ? "95%" : "110%",
              objectFit: "contain",
              pointerEvents: "none",
              userSelect: "none",
              transform:
                isActive || !isPressed ? "translateY(0px)" : "translateY(4px)",
              transition: "transform 0.12s ease",
            },
          })
        );
      })
    );
  }
);

// ClosetDrawer component (CUSTOM SCROLLBAR: track + thumb)
const ClosetDrawer = React.forwardRef(
  (
    {
      items,
      isOpen,
      itemRefs,
      scrollViewportRef,
      scrollTrackRef,
      scrollThumbRef,
      clearButtonRef,
      selectedTopName,
      selectedBottomName,
      selectedFullbodyName,
      selectedHatName,
      selectedAccessoryName,
      pageLabel,
    },
    ref
  ) => {
    useEffect(() => {
      itemRefs.current = {};
    }, [items, itemRefs]);

    return React.createElement(
      "div",
      {
        ref,
        id: "closet-drawer",
        className: isOpen ? "active" : "",
        style: {
          position: "absolute",
          top: DRAWER_TOP,
          right: 0,
          height: DRAWER_HEIGHT,
          width: `${DRAWER_WIDTH}px`,
          display: "flex",
          flexDirection: "column",
          gap: "0px",
          padding: "20px 10px",
          zIndex: 2,
          backgroundColor: "#b3c3daff",
          borderRadius: "16px 0 0 16px",
          backdropFilter: "blur(8px)",
          transition: "transform 0.4s ease-in-out, opacity 0.3s",
          transform: isOpen ? "translateX(0)" : "translateX(100%)",
          opacity: isOpen ? 1 : 0,
          overflow: "hidden",
          minHeight: 0,
        },
      },

      // Header (non-scrollable)
      React.createElement(
        "div",
        {
          style: {
            flex: "0 0 auto",
            display: "flex",
            flexDirection: "column",
            alignItems: "center",
            justifyContent: "flex-start",
            gap: `${DRAWER_HEADER_GAP}px`,
            height: `${DRAWER_HEADER_HEIGHT}px`,
            width: "100%",
            marginBottom: "8px",
            boxSizing: "border-box",
          },
        },

        // Logo
        React.createElement(
          "div",
          {
            style: {
              height: `${DRAWER_LOGO_HEIGHT}px`,
              width: "100%",
              overflow: "hidden",
              display: "flex",
              alignItems: "center",
              justifyContent: "center",
            },
          },
          React.createElement("img", {
            src: "/static/Glamcam.png",
            alt: "Glamcam",
            draggable: false,
            style: {
              height: "100%",
              width: "100%",
              objectFit: "contain",
              transform: "scale(1)",
              transformOrigin: "center",
              display: "block",
              pointerEvents: "none",
              userSelect: "none",
            },
          })
        ),

        // Clear button
        React.createElement("div", {
          ref: clearButtonRef,
          className: "clear-button",
          style: {
            height: `${DRAWER_CLEAR_HEIGHT}px`,
            width: `${DRAWER_CLEAR_HEIGHT}px`,
            borderRadius: "50%",
            backgroundColor: "rgba(255, 255, 255, 0.25)",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            cursor: "pointer",
            transition: "background-color 0.15s ease, transform 0.15s ease",
          },
          children: React.createElement("img", {
            src: "/static/clear.png",
            alt: "Reset",
            draggable: false,
            style: {
              height: "100%",
              width: "100%",
              objectFit: "contain",
              pointerEvents: "none",
              userSelect: "none",
            },
          }),
        }),

        React.createElement(
          "div",
          {
            className: "drawer-page-label",
            style: {
              height: `${DRAWER_PAGE_LABEL_HEIGHT}px`,

              // full-width bleed
              marginLeft: `-${DRAWER_PADDING_X}px`,
              marginRight: `-${DRAWER_PADDING_X}px`,
              width: `calc(100% + ${2 * DRAWER_PADDING_X}px)`,

              backgroundColor: "#DEEAF4",

              // text styling
              color: "#7685c5",
              fontFamily: `"Inter", "SF Pro Display", "Segoe UI", sans-serif`,
              fontSize: "16px",
              fontWeight: 600,
              letterSpacing: "0.3px",

              // shape
              borderRadius: "0px",

              display: "flex",
              alignItems: "center",
              justifyContent: "center",

              userSelect: "none",
              pointerEvents: "none",
              boxSizing: "border-box",
              padding: "0 10px",
            },
          },
          pageLabel || ""
        )
      ),

      React.createElement(
        "div",
        {
          style: {
            position: "relative",
            flex: "1 1 auto",
            minHeight: 0,
            width: "100%",
          },
        },

        // Scroll viewport (native scrollbar hidden)
        React.createElement(
          "div",
          {
            ref: scrollViewportRef,
            id: "drawer-scroll",
            style: {
              width: "100%",
              height: "100%",
              maxHeight: `${SCROLL_VIEW_HEIGHT}px`,
              overflowY: items.length > VISIBLE_ITEMS ? "auto" : "hidden",
              overflowX: "hidden",
              scrollbarWidth: "none",
            },
          },

          // Content wrapper (shifted right so it never overlaps track/thumb)
          React.createElement(
            "div",
            {
              style: {
                display: "flex",
                flexDirection: "column",
                gap: `${DRAWER_GAP}px`,
                width: "100%",
                paddingLeft: `${SCROLL_CONTENT_PAD_LEFT}px`,
                paddingRight: "2px",
                paddingTop: "4px",
                paddingBottom: "4px",
                boxSizing: "border-box",
              },
            },
            items.map((item) => {
              const isSelected =
                (item.type === "top" && selectedTopName === item.name) ||
                (item.type === "bottom" && selectedBottomName === item.name) ||
                (item.type === "fullbody" &&
                  selectedFullbodyName === item.name) ||
                (item.type === "hat" && selectedHatName === item.name) ||
                (item.type === "accessory" &&
                  selectedAccessoryName === item.name);

              return React.createElement(
                "div",
                {
                  key: item.name,
                  ref: (el) => {
                    if (el) itemRefs.current[item.name] = el;
                  },
                  className: `closet-item ${isSelected ? "selected" : ""}`,
                  "data-type": item.type,
                  "data-name": item.name,
                  style: {
                    width: "100%",
                    aspectRatio: "1 / 1",
                    borderRadius: "10px",
                    overflow: "hidden",
                    backgroundColor: "#ebe4daff",
                    display: "flex",
                    justifyContent: "center",
                    alignItems: "center",
                    position: "relative",
                    boxSizing: "border-box",
                    padding: "6px",
                    transition: "none",
                  },
                },
                React.createElement("div", {
                  style: {
                    position: "absolute",
                    inset: 0,
                    borderRadius: "10px",
                    pointerEvents: "none",
                    opacity: isSelected ? 1 : 0,
                    transition: "opacity 0.15s ease",
                    background:
                      "radial-gradient(circle at center, rgba(255,255,255,0.95) 0%, rgba(255,255,255,0.55) 35%, rgba(251,215,222,0.95) 78%, rgba(251,215,222,0.95) 100%)",
                  },
                }),
                React.createElement("div", {
                  style: {
                    position: "absolute",
                    inset: 0,
                    borderRadius: "10px",
                    pointerEvents: "none",
                    opacity: isSelected ? 1 : 0,
                    transition: "opacity 0.15s ease",
                    boxShadow: "inset 0 0 0 4px rgba(255,255,255,0.95)",
                  },
                }),
                React.createElement("img", {
                  src: item.image,
                  alt: item.label,
                  style: {
                    width: "100%",
                    height: "100%",
                    objectFit: "contain",
                    position: "relative",
                    zIndex: 1,
                  },
                })
              );
            })
          )
        ),

        // Hide native scrollbar (WebKit)
        React.createElement("style", {
          dangerouslySetInnerHTML: {
            __html: `
              #drawer-scroll::-webkit-scrollbar { width: 0px; height: 0px; }
            `,
          },
        }),

        // Custom scrollbar track (left edge)
        React.createElement(
          "div",
          {
            ref: scrollTrackRef,
            className: "drawer-scrollbar-track",
            style: {
              position: "absolute",
              left: `${SCROLL_TRACK_INSET}px`,
              top: 0,
              bottom: 0,
              width: `${SCROLL_TRACK_WIDTH}px`,
              borderRadius: "999px",
              background: "rgba(255,255,255,0.12)",
              zIndex: 6,
              pointerEvents: isOpen ? "auto" : "none",
              transition: "background-color 0.15s ease",
            },
          },
          React.createElement("div", {
            ref: scrollThumbRef,
            className: "drawer-scrollbar-thumb",
            style: {
              position: "absolute",
              left: "3px",
              right: "3px",
              top: "0px",
              height: "28px",
              borderRadius: "999px",
              background: "rgba(255,255,255,0.35)",
              transition: "background-color 0.15s ease, opacity 0.15s ease",
              opacity: 0,
            },
          })
        )
      )
    );
  }
);

// VirtualCursor component
function VirtualCursor({ x, y }) {
  return React.createElement("div", {
    id: "virtual-cursor",
    style: {
      position: "fixed",
      pointerEvents: "none",
      zIndex: 100,
      width: "36px",
      height: "36px",
      borderRadius: "50%",
      background: "#ffd600",
      border: "none",
      boxShadow: "0 0 12px 2px #ffd60077",
      transform: "translate(-18px, -18px)",
      left: `${x}px`,
      top: `${y}px`,
    },
  });
}

// useKinectWebSocket hook
function useKinectWebSocket(onMessage) {
  const wsRef = useRef(null);
  const reconnectTimerRef = useRef(null);

  useEffect(() => {
    function connect() {
      const ws = new WebSocket(`ws://${window.location.hostname}:8765`);

      ws.onopen = () => console.log("Kinect WebSocket connected");

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          onMessage(data);
        } catch (e) {
          console.error("Error parsing Kinect data:", e);
        }
      };

      ws.onclose = () => {
        console.log("Kinect WebSocket disconnected. Reconnecting in 2s...");
        if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
        reconnectTimerRef.current = setTimeout(connect, 2000);
      };

      ws.onerror = (e) => {
        console.error("Kinect WebSocket error:", e);
        ws.close();
      };

      wsRef.current = ws;
    }

    connect();

    return () => {
      if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
      if (wsRef.current) wsRef.current.close();
    };
  }, [onMessage]);
}

// useRunIdMonitor hook
function useRunIdMonitor() {
  useEffect(() => {
    let last = null;

    async function tick() {
      try {
        const r = await fetch("/run_id", { cache: "no-store" });
        const id = await r.text();

        if (last === null) last = id;
        else if (id !== last) location.reload();
      } catch (e) {}
      setTimeout(tick, 500);
    }

    tick();
  }, []);
}

// Main App component
function App() {
  const [cursorX, setCursorX] = useState(window.innerWidth / 2);
  const [cursorY, setCursorY] = useState(window.innerHeight / 2);

  const [closetOpen, setClosetOpen] = useState(false);

  // which tab segment is active (controls drawer page)
  const [activeCategoryIndex, setActiveCategoryIndex] = useState(null);

  const activeCategoryLabel =
    activeCategoryIndex === null
      ? null
      : (CLOSET_CATEGORIES[activeCategoryIndex] &&
          CLOSET_CATEGORIES[activeCategoryIndex].key) ||
        null;

  // drawer page label for header section
  const drawerPageLabel = activeCategoryLabel || "";

  const activeItems =
    activeCategoryLabel === null
      ? []
      : CLOTHING_PAGES[activeCategoryLabel] || [];
  const activeItemsCountRef = useRef(activeItems.length);

  const [selectedTopName, setSelectedTopName] = useState(null);
  const [selectedBottomName, setSelectedBottomName] = useState(null);
  const [selectedFullbodyName, setSelectedFullbodyName] = useState(null);
  const [selectedHatName, setSelectedHatName] = useState(null);
  const [selectedAccessoryName, setSelectedAccessoryName] = useState(null);
  const [useKinect, setUseKinect] = useState(false);

  const [pressedSegIndex, setPressedSegIndex] = useState(null);
  const pressedSegIndexRef = useRef(null);

  useEffect(() => {
    pressedSegIndexRef.current = pressedSegIndex;
  }, [pressedSegIndex]);

  const hoverTargetRef = useRef(null);
  const hoverStartTimeRef = useRef(null);

  const closetTabRef = useRef(null);
  const closetTabSegmentRefs = useRef([]);
  const closetDrawerRef = useRef(null);
  const clearButtonRef = useRef(null);
  const closetItemRefs = useRef({});

  // Custom scrollbar refs
  const drawerScrollViewportRef = useRef(null);
  const drawerScrollTrackRef = useRef(null);
  const drawerScrollThumbRef = useRef(null);

  // Drag state
  const scrollDraggingRef = useRef(false);
  const scrollDragOffsetYRef = useRef(0);

  // Auto-close timer refs
  const lastInsideUiAtRef = useRef(performance.now());
  const autoCloseFiredRef = useRef(false);

  const cursorRef = useRef({ x: cursorX, y: cursorY });
  const closetOpenRef = useRef(closetOpen);
  const rafIdRef = useRef(null);
  const stoppedRef = useRef(false);

  const selectedTopNameRef = useRef(selectedTopName);
  const selectedBottomNameRef = useRef(selectedBottomName);
  const selectedFullbodyNameRef = useRef(selectedFullbodyName);
  const selectedHatNameRef = useRef(selectedHatName);
  const selectedAccessoryNameRef = useRef(selectedAccessoryName);

  useEffect(() => {
    cursorRef.current = { x: cursorX, y: cursorY };
  }, [cursorX, cursorY]);

  useEffect(() => {
    closetOpenRef.current = closetOpen;

    // If drawer closes, force stop dragging
    if (!closetOpen) {
      scrollDraggingRef.current = false;
      autoCloseFiredRef.current = false; // reset when closed
    }
  }, [closetOpen]);

  useEffect(() => {
    activeItemsCountRef.current = activeItems.length;
  }, [activeItems.length]);

  useEffect(() => {
    selectedTopNameRef.current = selectedTopName;
  }, [selectedTopName]);

  useEffect(() => {
    selectedBottomNameRef.current = selectedBottomName;
  }, [selectedBottomName]);

  useEffect(() => {
    selectedFullbodyNameRef.current = selectedFullbodyName;
  }, [selectedFullbodyName]);

  useEffect(() => {
    selectedHatNameRef.current = selectedHatName;
  }, [selectedHatName]);

  useEffect(() => {
    selectedAccessoryNameRef.current = selectedAccessoryName;
  }, [selectedAccessoryName]);

  useRunIdMonitor();

  const handleKinectData = useCallback((data) => {
    if (typeof data.x === "number" && typeof data.y === "number") {
      const windowWidth = window.innerWidth;
      const windowHeight = window.innerHeight;
      const frameWidth = 1920;
      const frameHeight = 1080;

      const leftCut = (frameWidth - windowWidth) / 2;
      const topCut = (frameHeight - windowHeight) / 2;

      const x = data.x - leftCut;
      const y = data.y - topCut;

      if (x < 0 || x > windowWidth || y < 0 || y > windowHeight) {
        setUseKinect(false);
      } else {
        setUseKinect(true);
        setCursorX(x);
        setCursorY(y);
      }
    }
  }, []);

  useKinectWebSocket(handleKinectData);

  useEffect(() => {
    const handleMouseMove = (e) => {
      if (!useKinect) {
        setCursorX(e.clientX);
        setCursorY(e.clientY);
      }
    };
    document.addEventListener("mousemove", handleMouseMove);
    return () => document.removeEventListener("mousemove", handleMouseMove);
  }, [useKinect]);

  const circleRectCollision = useCallback((cx, cy, radius, rect) => {
    const rx = Math.max(rect.left, Math.min(cx, rect.right));
    const ry = Math.max(rect.top, Math.min(cy, rect.bottom));
    const dx = cx - rx;
    const dy = cy - ry;
    return dx * dx + dy * dy <= radius * radius;
  }, []);

  const sendSelectionToBackend = useCallback((type, action, name) => {
    fetch("/select", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ type, action, name }),
    });
  }, []);

  // Sync thumb size + position to current scrollTop
  const syncThumb = useCallback(() => {
    const scroller = drawerScrollViewportRef.current;
    const track = drawerScrollTrackRef.current;
    const thumb = drawerScrollThumbRef.current;
    if (!scroller || !track || !thumb) return;

    const trackRect = track.getBoundingClientRect();

    const scrollHeight = scroller.scrollHeight;
    const clientHeight = scroller.clientHeight;

    if (scrollHeight <= clientHeight + 1) {
      thumb.style.opacity = "0";
      thumb.style.height = `${Math.max(28, trackRect.height)}px`;
      thumb.style.top = "0px";
      return;
    }

    thumb.style.opacity = "1";

    const minThumbH = 28;
    const thumbH = Math.max(
      minThumbH,
      Math.round((clientHeight / scrollHeight) * trackRect.height)
    );
    const maxThumbTop = Math.max(1, trackRect.height - thumbH);

    const maxScrollTop = Math.max(1, scrollHeight - clientHeight);
    const ratio = scroller.scrollTop / maxScrollTop;
    const thumbTop = Math.round(ratio * maxThumbTop);

    thumb.style.height = `${thumbH}px`;
    thumb.style.top = `${thumbTop}px`;
  }, []);

  // Keep thumb synced when list scrolls (wheel/drag/programmatic)
  useEffect(() => {
    const scroller = drawerScrollViewportRef.current;
    if (!scroller) return;

    const onScroll = () => syncThumb();
    scroller.addEventListener("scroll", onScroll, { passive: true });
    return () => scroller.removeEventListener("scroll", onScroll);
  }, [syncThumb]);

  // Re-sync on drawer open / category change / item length change
  useEffect(() => {
    // Optionally reset to top when changing category
    const scroller = drawerScrollViewportRef.current;
    if (scroller) scroller.scrollTop = 0;
    syncThumb();
  }, [activeCategoryIndex, activeItems.length, closetOpen, syncThumb]);

  const handleDwellSelect = useCallback(
    (target) => {
      setPressedSegIndex(null);

      // Clear button dwell - deselect all
      if (target === clearButtonRef.current) {
        setSelectedTopName(null);
        setSelectedBottomName(null);
        setSelectedFullbodyName(null);
        setSelectedHatName(null);
        setSelectedAccessoryName(null);
        sendSelectionToBackend("all", "clear", "");
        return;
      }

      // Thumb dwell toggles drag ON/OFF
      if (target === drawerScrollThumbRef.current) {
        if (!closetOpenRef.current) return;

        if (!scrollDraggingRef.current) {
          const thumbRect =
            drawerScrollThumbRef.current.getBoundingClientRect();
          scrollDraggingRef.current = true;
          scrollDragOffsetYRef.current = cursorRef.current.y - thumbRect.top;
        } else {
          scrollDraggingRef.current = false;
        }
        return;
      }

      // Track is hover-highlight only; not selectable (per your requirement)
      if (target === drawerScrollTrackRef.current) return;

      // Segment dwell
      if (target?.classList?.contains("closet-tab-segment")) {
        const idxStr = target.getAttribute("data-seg-index");
        const idx = idxStr ? parseInt(idxStr, 10) : 0;

        setActiveCategoryIndex(idx);
        if (!closetOpenRef.current) setClosetOpen(true);
        return;
      }

      // Item dwell
      if (closetOpenRef.current && target?.classList?.contains("closet-item")) {
        const type = target.getAttribute("data-type");
        const name = target.getAttribute("data-name");

        if (type === "top") {
          if (selectedTopNameRef.current === name) {
            setSelectedTopName(null);
            sendSelectionToBackend(type, "deselect", name);
          } else {
            if (selectedFullbodyNameRef.current) {
              sendSelectionToBackend(
                "fullbody",
                "deselect",
                selectedFullbodyNameRef.current
              );
              setSelectedFullbodyName(null);
            }
            if (selectedTopNameRef.current) {
              sendSelectionToBackend(
                "top",
                "deselect",
                selectedTopNameRef.current
              );
            }
            setSelectedTopName(name);
            sendSelectionToBackend(type, "select", name);
          }
          return;
        }

        if (type === "bottom") {
          if (selectedBottomNameRef.current === name) {
            setSelectedBottomName(null);
            sendSelectionToBackend(type, "deselect", name);
          } else {
            if (selectedFullbodyNameRef.current) {
              sendSelectionToBackend(
                "fullbody",
                "deselect",
                selectedFullbodyNameRef.current
              );
              setSelectedFullbodyName(null);
            }
            if (selectedBottomNameRef.current) {
              sendSelectionToBackend(
                "bottom",
                "deselect",
                selectedBottomNameRef.current
              );
            }
            setSelectedBottomName(name);
            sendSelectionToBackend(type, "select", name);
          }
          return;
        }

        if (type === "fullbody") {
          if (selectedFullbodyNameRef.current === name) {
            setSelectedFullbodyName(null);
            sendSelectionToBackend(type, "deselect", name);
          } else {
            if (selectedTopNameRef.current) {
              sendSelectionToBackend(
                "top",
                "deselect",
                selectedTopNameRef.current
              );
              setSelectedTopName(null);
            }
            if (selectedBottomNameRef.current) {
              sendSelectionToBackend(
                "bottom",
                "deselect",
                selectedBottomNameRef.current
              );
              setSelectedBottomName(null);
            }
            if (selectedFullbodyNameRef.current) {
              sendSelectionToBackend(
                "fullbody",
                "deselect",
                selectedFullbodyNameRef.current
              );
            }
            setSelectedFullbodyName(name);
            sendSelectionToBackend(type, "select", name);
          }
          return;
        }

        if (type === "hat") {
          if (selectedHatNameRef.current === name) {
            setSelectedHatName(null);
            sendSelectionToBackend(type, "deselect", name);
          } else {
            if (selectedHatNameRef.current) {
              sendSelectionToBackend(
                "hat",
                "deselect",
                selectedHatNameRef.current
              );
            }
            setSelectedHatName(name);
            sendSelectionToBackend(type, "select", name);
          }
          return;
        }

        if (type === "accessory") {
          if (selectedAccessoryNameRef.current === name) {
            setSelectedAccessoryName(null);
            sendSelectionToBackend(type, "deselect", name);
          } else {
            if (selectedAccessoryNameRef.current) {
              sendSelectionToBackend(
                "accessory",
                "deselect",
                selectedAccessoryNameRef.current
              );
            }
            setSelectedAccessoryName(name);
            sendSelectionToBackend(type, "select", name);
          }
          return;
        }
      }
    },
    [sendSelectionToBackend]
  );

  // Single RAF hover loop (custom scrollbar drag + hover)
  useEffect(() => {
    stoppedRef.current = false;

    const checkHover = () => {
      if (stoppedRef.current) return;

      const { x, y } = cursorRef.current;
      const cursorRadius = 24;

      // If dragging thumb, update scrollTop based on cursor Y
      if (scrollDraggingRef.current) {
        const scroller = drawerScrollViewportRef.current;
        const track = drawerScrollTrackRef.current;
        const thumb = drawerScrollThumbRef.current;

        if (scroller && track && thumb) {
          const trackRect = track.getBoundingClientRect();
          const thumbRect = thumb.getBoundingClientRect();

          // If cursor leaves the thumb while dragging, release it.
          const stillOverThumb = circleRectCollision(
            x,
            y,
            cursorRadius,
            thumbRect
          );
          if (!stillOverThumb) {
            scrollDraggingRef.current = false;
            rafIdRef.current = requestAnimationFrame(checkHover);
            return;
          }

          const trackH = trackRect.height;
          const thumbH = thumbRect.height;
          const maxThumbTop = Math.max(1, trackH - thumbH);

          const desiredThumbTop =
            y - trackRect.top - scrollDragOffsetYRef.current;
          const clampedThumbTop = Math.max(
            0,
            Math.min(desiredThumbTop, maxThumbTop)
          );

          const maxScrollTop = Math.max(
            1,
            scroller.scrollHeight - scroller.clientHeight
          );
          const ratio = clampedThumbTop / maxThumbTop;

          scroller.scrollTop = ratio * maxScrollTop;
          syncThumb();
        }

        rafIdRef.current = requestAnimationFrame(checkHover);
        return;
      }

      let foundTarget = null;

      // 1) Tab segments
      for (const seg of closetTabSegmentRefs.current) {
        if (!seg) continue;
        const rect = seg.getBoundingClientRect();
        if (circleRectCollision(x, y, cursorRadius, rect)) {
          foundTarget = seg;
          break;
        }
      }

      // 2) Thumb (highest priority)
      if (closetOpenRef.current && drawerScrollThumbRef.current) {
        const rect = drawerScrollThumbRef.current.getBoundingClientRect();
        if (circleRectCollision(x, y, cursorRadius, rect)) {
          foundTarget = drawerScrollThumbRef.current;
        }
      }

      // 3) Track (next priority)
      if (
        !foundTarget &&
        closetOpenRef.current &&
        drawerScrollTrackRef.current
      ) {
        const rect = drawerScrollTrackRef.current.getBoundingClientRect();
        if (circleRectCollision(x, y, cursorRadius, rect)) {
          foundTarget = drawerScrollTrackRef.current;
        }
      }

      // 4) Clear button
      if (!foundTarget && closetOpenRef.current && clearButtonRef.current) {
        const rect = clearButtonRef.current.getBoundingClientRect();
        if (circleRectCollision(x, y, cursorRadius, rect)) {
          foundTarget = clearButtonRef.current;
        }
      }

      // 5) Items
      if (!foundTarget && closetOpenRef.current && closetDrawerRef.current) {
        for (const item of Object.values(closetItemRefs.current)) {
          if (!item) continue;
          const rect = item.getBoundingClientRect();
          if (circleRectCollision(x, y, cursorRadius, rect)) {
            foundTarget = item;
            break;
          }
        }
      }

      // Hover visuals for clear button
      if (clearButtonRef.current) {
        const hoveringClear = foundTarget === clearButtonRef.current;
        clearButtonRef.current.style.backgroundColor = hoveringClear
          ? "rgba(255, 255, 255, 0.5)"
          : "rgba(255, 255, 255, 0.25)";
      }

      // Hover visuals for track + thumb
      if (drawerScrollTrackRef.current) {
        const hoveringTrack =
          foundTarget === drawerScrollTrackRef.current ||
          foundTarget === drawerScrollThumbRef.current;
        drawerScrollTrackRef.current.style.backgroundColor = hoveringTrack
          ? "rgba(255,255,255,0.25)"
          : "rgba(255,255,255,0.12)";
      }
      if (drawerScrollThumbRef.current) {
        const hoveringThumb = foundTarget === drawerScrollThumbRef.current;
        drawerScrollThumbRef.current.style.backgroundColor = hoveringThumb
          ? "rgba(255,255,255,0.65)"
          : "rgba(255,255,255,0.35)";
      }

      // Press animation for tab segments (while dwelling/hovering)
      let hoveringSegIndex = null;
      if (foundTarget?.classList?.contains("closet-tab-segment")) {
        const idxStr = foundTarget.getAttribute("data-seg-index");
        hoveringSegIndex = idxStr ? parseInt(idxStr, 10) : 0;
      }

      if (hoveringSegIndex !== pressedSegIndexRef.current) {
        setPressedSegIndex(hoveringSegIndex);
      }

      // AUTO CLOSE: if cursor is outside tab + TAB_GAP corridor + drawer for 2s, close
      const now = performance.now();

      const overTab = (() => {
        const el = closetTabRef.current;
        if (!el) return false;
        const r = el.getBoundingClientRect();
        return circleRectCollision(x, y, cursorRadius, r);
      })();

      const overDrawer = (() => {
        if (!closetOpenRef.current) return false;
        const el = closetDrawerRef.current;
        if (!el) return false;
        const r = el.getBoundingClientRect();
        return circleRectCollision(x, y, cursorRadius, r);
      })();

      // Corridor between tab and drawer (the TAB_GAP space), only meaningful when drawer is open.
      const overTabGap = (() => {
        if (!closetOpenRef.current) return false;
        const tabEl = closetTabRef.current;
        const drawerEl = closetDrawerRef.current;
        if (!tabEl || !drawerEl) return false;

        const tr = tabEl.getBoundingClientRect();
        const dr = drawerEl.getBoundingClientRect();

        // Create a rectangle spanning the horizontal gap between tab (left edge) and drawer (right edge)
        const left = Math.min(dr.right, tr.left); // drawer right is near screen edge
        const right = Math.max(dr.left, tr.right); // corridor between tr.right and dr.left

        // Only treat as gap if there's an actual gap
        if (right <= left + 1) return false;

        const gapRect = {
          left,
          right,
          top: Math.min(tr.top, dr.top),
          bottom: Math.max(tr.bottom, dr.bottom),
        };

        return circleRectCollision(x, y, cursorRadius, gapRect);
      })();

      const insideUi =
        overTab || overDrawer || overTabGap || foundTarget != null;

      if (insideUi) {
        lastInsideUiAtRef.current = now;
        autoCloseFiredRef.current = false;
      } else {
        if (
          closetOpenRef.current &&
          !autoCloseFiredRef.current &&
          now - lastInsideUiAtRef.current >= AUTO_CLOSE_MS
        ) {
          autoCloseFiredRef.current = true;

          // Close drawer + reset page selection/hover state
          setClosetOpen(false);
          setActiveCategoryIndex(null);
          setPressedSegIndex(null);

          // stop any hover/dwell-in-progress immediately
          hoverTargetRef.current = null;
          hoverStartTimeRef.current = null;

          // stop scroll drag immediately
          scrollDraggingRef.current = false;
        }
      }

      // Dwell timing
      if (foundTarget !== hoverTargetRef.current) {
        hoverTargetRef.current = foundTarget;
        hoverStartTimeRef.current = foundTarget ? performance.now() : null;
      } else if (hoverTargetRef.current && hoverStartTimeRef.current) {
        const elapsed = performance.now() - hoverStartTimeRef.current;
        if (elapsed >= DWELL_TIME) {
          handleDwellSelect(hoverTargetRef.current);
          hoverStartTimeRef.current = null;
        }
      }

      rafIdRef.current = requestAnimationFrame(checkHover);
    };

    rafIdRef.current = requestAnimationFrame(checkHover);

    return () => {
      stoppedRef.current = true;
      if (rafIdRef.current) cancelAnimationFrame(rafIdRef.current);
    };
  }, [circleRectCollision, handleDwellSelect, syncThumb, setPressedSegIndex]);

  return React.createElement(
    "div",
    { id: "video-container" },
    React.createElement(VideoFeed),
    React.createElement(ClosetTab, {
      ref: closetTabRef,
      isOpen: closetOpen,
      activeIndex: activeCategoryIndex,
      pressedIndex: pressedSegIndex,
      segmentRefs: closetTabSegmentRefs,
    }),
    React.createElement(ClosetDrawer, {
      ref: closetDrawerRef,
      items: activeItems,
      isOpen: closetOpen,
      itemRefs: closetItemRefs,
      scrollViewportRef: drawerScrollViewportRef,
      scrollTrackRef: drawerScrollTrackRef,
      scrollThumbRef: drawerScrollThumbRef,
      clearButtonRef: clearButtonRef,
      selectedTopName,
      selectedBottomName,
      selectedFullbodyName,
      selectedHatName,
      selectedAccessoryName,
      pageLabel: drawerPageLabel,
    }),
    React.createElement(VirtualCursor, { x: cursorX, y: cursorY })
  );
}

ReactDOM.render(React.createElement(App), document.getElementById("root"));
