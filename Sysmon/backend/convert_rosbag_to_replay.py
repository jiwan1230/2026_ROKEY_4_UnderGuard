#!/usr/bin/env python3
"""rosbag2 디렉터리를 System Monitor Replay JSON 파일로 변환하는 명령어."""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path

from system_monitor.rosbag_converter import (
    ConversionError,
    TopicConfig,
    build_replay_document,
    build_trial,
    read_rosbag_events,
)


def _optional_topic(value: str) -> str | None:
    value = value.strip()
    return value or None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="rosbag2의 쥐몰이 토픽을 System Monitor Replay JSON으로 변환합니다."
    )
    parser.add_argument("bag", type=Path, help="metadata.yaml이 들어 있는 rosbag2 디렉터리")
    parser.add_argument("output", type=Path, help="생성할 Replay JSON 파일")
    parser.add_argument("--driver-id", default="robot4")
    parser.add_argument("--blocker-id", default="robot6")
    parser.add_argument("--driver-odom", default="/robot4/odom")
    parser.add_argument("--blocker-odom", default="/robot6/odom")
    parser.add_argument("--target-event", default="/fleet/event")
    parser.add_argument("--target-pose", default="")
    parser.add_argument("--driver-goal", default="/robot4/target_pose")
    parser.add_argument("--blocker-goal", default="/robot6/target_pose")
    parser.add_argument("--fleet-status", default="/fleet/status")
    parser.add_argument("--state-topic", default="/herding/state")
    parser.add_argument("--progress-topic", default="/herding/capture_progress")
    parser.add_argument("--success-topic", default="/herding/success")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--sample-period", type=float, default=0.1, help="출력 프레임 간격(초)")
    parser.add_argument("--model", default="rosbag_recording", help="시험에 표시할 알고리즘 모델명")
    parser.add_argument("--goal-name", default=None, help="포획 지점 이름(top/left/bottom 등)")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--base-replay",
        type=Path,
        help="지도 이미지·포획 지점을 복사할 기존 Replay JSON",
    )
    parser.add_argument(
        "--append-existing-trials",
        action="store_true",
        help="base-replay의 기존 trials 뒤에 새 시험을 추가",
    )
    parser.add_argument("--force", action="store_true", help="출력 파일이 있어도 덮어쓰기")
    return parser


def _read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ConversionError(f"기준 Replay JSON을 읽을 수 없습니다: {path}") from error


def _write_json_atomic(path: Path, document: dict) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
        ) as temp:
            json.dump(document, temp, ensure_ascii=False, indent=2)
            temp.write("\n")
            temporary_path = Path(temp.name)
        os.replace(temporary_path, path)
    except OSError as error:
        raise ConversionError(f"Replay JSON을 저장할 수 없습니다: {path}") from error


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.output.exists() and not args.force:
        print(f"오류: 출력 파일이 이미 있습니다: {args.output} (--force로 덮어쓰기)", file=sys.stderr)
        return 2
    if args.append_existing_trials and args.base_replay is None:
        print("오류: --append-existing-trials에는 --base-replay가 필요합니다.", file=sys.stderr)
        return 2

    topics = TopicConfig(
        driver_id=args.driver_id,
        blocker_id=args.blocker_id,
        driver_odom=args.driver_odom,
        blocker_odom=args.blocker_odom,
        target_event=_optional_topic(args.target_event),
        target_pose=_optional_topic(args.target_pose),
        driver_goal=_optional_topic(args.driver_goal),
        blocker_goal=_optional_topic(args.blocker_goal),
        fleet_status=_optional_topic(args.fleet_status),
        state=_optional_topic(args.state_topic),
        capture_progress=_optional_topic(args.progress_topic),
        success=_optional_topic(args.success_topic),
    )
    try:
        events, warnings, topic_types = read_rosbag_events(
            args.bag, topics, map_frame=args.map_frame
        )
        trial = build_trial(
            events,
            sample_period=args.sample_period,
            model=args.model,
            goal_name=args.goal_name,
            seed=args.seed,
        )
        base = _read_json(args.base_replay) if args.base_replay else None
        document = build_replay_document(
            trial,
            base_document=base,
            append_existing_trials=args.append_existing_trials,
        )
        document["conversion"] = {
            "source_bag": str(args.bag),
            "map_frame": args.map_frame.strip("/") or "map",
            "sample_period": args.sample_period,
            "topics": as_topic_dict(topics),
            "bag_topic_types": topic_types,
            "warnings": warnings,
        }
        _write_json_atomic(args.output, document)
    except ConversionError as error:
        print(f"오류: {error}", file=sys.stderr)
        return 2

    print(
        f"변환 완료: {args.output} (프레임 {len(trial['frames'])}개, "
        f"{trial['duration']:.1f}초, 성공={trial['success']})"
    )
    for warning in warnings:
        print(f"주의: {warning}")
    return 0


def as_topic_dict(topics: TopicConfig) -> dict[str, str | None]:
    """TopicConfig를 JSON에 기록할 단순 dict로 바꾼다."""

    normalized = topics.normalized()
    return {
        name: getattr(normalized, name)
        for name in normalized.__dataclass_fields__
    }


if __name__ == "__main__":
    raise SystemExit(main())
