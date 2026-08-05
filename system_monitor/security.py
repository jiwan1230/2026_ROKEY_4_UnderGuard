"""PBKDF2 기반 로그인 비밀번호 생성과 검증을 담당한다."""

from __future__ import annotations

import hashlib
import hmac
import os

_ITERATIONS = 240_000


def hash_password(password: str) -> str:
    """평문 비밀번호를 저장용 해시 문자열로 변환한다.

    입력: 8자 이상의 평문 비밀번호다.
    출력: 알고리즘·반복 횟수·salt·digest가 결합된 문자열이다.
    사용: 사용자 생성 시 결과만 DB에 저장하고 평문은 저장하지 않는다.
    """

    if len(password) < 8:
        raise ValueError("비밀번호는 8자 이상이어야 합니다.")
    salt = os.urandom(16)
    digest = hashlib.pbkdf2_hmac("sha256", password.encode(), salt, _ITERATIONS)
    return f"pbkdf2_sha256${_ITERATIONS}${salt.hex()}${digest.hex()}"


def verify_password(password: str, encoded: str) -> bool:
    """평문과 저장된 해시를 상수 시간으로 비교한다.

    입력: 로그인 평문과 ``hash_password``가 만든 저장 문자열이다.
    출력: 일치하면 ``True``이며 형식이 잘못됐거나 불일치하면 ``False``다.
    사용: 로그인 라우트에서 DB 사용자 조회 후 호출한다.
    """

    try:
        algorithm, iterations, salt_hex, digest_hex = encoded.split("$", 3)
        if algorithm != "pbkdf2_sha256":
            return False
        candidate = hashlib.pbkdf2_hmac(
            "sha256",
            password.encode(),
            bytes.fromhex(salt_hex),
            int(iterations),
        )
        return hmac.compare_digest(candidate.hex(), digest_hex)
    except (ValueError, TypeError):
        return False
