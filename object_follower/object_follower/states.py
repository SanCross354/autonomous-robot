#!/usr/bin/env python3
"""
states.py - State definitions for Object Follower

Defines the state machine states and transition logic.
Based on the hybrid architecture from KCF-YOLO paper.
"""

from enum import Enum, auto


class FollowerState(str, Enum):
    """
    State machine for object following.
    
    State Hierarchy (from KCF-YOLO paper):
    - IDLE: Waiting for object selection
    - EXPLORING: Navigating to waypoints using Nav2 (autonomous discovery)
    - SEARCHING: Rotating/moving to find lost object
    - APPROACHING: Nav2 approach to visible but far object (hybrid)
    - TRACKING: Pure visual servoing for close object
    - STOPPED: Object reached, maintaining position
    """
    IDLE = "IDLE"
    EXPLORING = "EXPLORING"
    SEARCHING = "SEARCHING"
    APPROACHING = "APPROACHING"  # Hybrid state: Nav2 approach to visible object
    TRACKING = "TRACKING"
    STOPPED = "STOPPED"


def can_transition(from_state: FollowerState, to_state: FollowerState) -> bool:
    """
    Validate state transitions.
    
    Returns True if the transition is allowed.
    """
    # Define allowed transitions
    allowed_transitions = {
        FollowerState.IDLE: [
            FollowerState.EXPLORING,
            FollowerState.SEARCHING,
        ],
        FollowerState.EXPLORING: [
            FollowerState.APPROACHING,  # Object detected far
            FollowerState.TRACKING,     # Object detected close
            FollowerState.SEARCHING,    # Exploration timeout/failure
            FollowerState.IDLE,         # User stop
        ],
        FollowerState.SEARCHING: [
            FollowerState.TRACKING,     # Object found
            FollowerState.APPROACHING,  # Object found far
            FollowerState.EXPLORING,    # Search timeout -> resume exploration
            FollowerState.IDLE,         # User stop
        ],
        FollowerState.APPROACHING: [
            FollowerState.TRACKING,     # Object now close
            FollowerState.SEARCHING,    # Object lost
            FollowerState.EXPLORING,    # User requested exploration
            FollowerState.IDLE,         # User stop
        ],
        FollowerState.TRACKING: [
            FollowerState.STOPPED,      # Object reached
            FollowerState.SEARCHING,    # Object lost
            FollowerState.APPROACHING,  # Object moved far
            FollowerState.IDLE,         # User stop
        ],
        FollowerState.STOPPED: [
            FollowerState.SEARCHING,    # Object moved/lost
            FollowerState.TRACKING,     # Object still visible
            FollowerState.IDLE,         # User stop
        ],
    }
    
    return to_state in allowed_transitions.get(from_state, [])


def get_state_priority(state: FollowerState) -> int:
    """
    Get priority for state (higher = more urgent).
    Used for conflict resolution.
    """
    priorities = {
        FollowerState.IDLE: 0,
        FollowerState.EXPLORING: 1,
        FollowerState.SEARCHING: 2,
        FollowerState.APPROACHING: 3,
        FollowerState.TRACKING: 4,
        FollowerState.STOPPED: 5,
    }
    return priorities.get(state, 0)
