import hmac, hashlib, secrets
from datetime import datetime, timezone

class Authorization:
    """
    현대 엘리베이터  Tings API 로봇연동 사물 원격 콜 요청 api 인증
    """
    def __init__(self, api_key: str, api_secret_hex: str):
        self.api_key = api_key
        self.api_secret_hex = api_secret_hex
        self.utf8 = "utf-8"
        self.hmac_algorithm = "sha256"
        self.__signature_method = "HmacSHA256"

    def get_signature_method(self)->str:
        return self.__signature_method
    def generate_nonce(self) -> str:
        return secrets.token_bytes(16).hex()

    def generate_timestamp(self) -> str:
        return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"

    def hmac_converter(self, key_bytes: bytes, data_bytes: bytes) -> bytes:
        """hmac.new(key, data, algorithm)에서 algorithm은 hashlib 모듈의 해시 알고리즘을 사용
        예제:
            key = b'secret_key'
            data = b'message'
            algorithm = 'sha256'
        """
        return hmac.new(key_bytes, data_bytes, getattr(hashlib, self.hmac_algorithm)).digest()

    def generate_signature(self, nonce_hex: str, timestamp: str) -> str:
        api_secret_bytes = bytes.fromhex(self.api_secret_hex)
        nonce_bytes = bytes.fromhex(nonce_hex)
        timestamp_bytes = timestamp.encode(self.utf8)
        api_key_bytes = self.api_key.encode(self.utf8)

        hmac_nonce_bytes = self.hmac_converter(api_secret_bytes, nonce_bytes)
        hmac_date_bytes = self.hmac_converter(hmac_nonce_bytes, timestamp_bytes)
        hmac_signature_bytes = self.hmac_converter(hmac_date_bytes, api_key_bytes)
        return hmac_signature_bytes.hex().lower()

