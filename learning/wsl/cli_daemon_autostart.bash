# Use UDP transport in WSL; avoid the unreliable shared-memory path.
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/ros/fastdds_udp_only.xml"

# Start the standard ROS CLI daemon before graph commands.
# Avoid the stalled connection probe when the daemon is absent in WSL.
ros2() {
    if ! type -P ros2 >/dev/null 2>&1; then
        command ros2 "$@"
        return $?
    fi

    # Graph lists use direct discovery instead of a just-started daemon cache.
    case "${1-} ${2-}" in
        "node list"|"topic list")
            local graph_verb="$1"
            shift 2
            local has_spin_time=0
            local argument
            for argument in "$@"; do
                case "$argument" in
                    --spin-time|--spin-time=*) has_spin_time=1 ;;
                esac
            done
            if [ "$has_spin_time" -eq 1 ]; then
                command ros2 "$graph_verb" list --no-daemon "$@"
            else
                command ros2 "$graph_verb" list --no-daemon --spin-time 3 "$@"
            fi
            return $?
            ;;
    esac

    case "${1-}" in
        node|topic|service|action|param)
            case " $* " in
                *" --no-daemon "*) ;;
                *)
                    /usr/bin/python3 -c 'from argparse import Namespace; from ros2cli.node.daemon import spawn_daemon; spawn_daemon(Namespace(), timeout=5)' || return $?
                    ;;
            esac
            ;;
    esac

    command ros2 "$@"
}
