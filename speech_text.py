"""Plain words for the chassis TTS setup, which mishandles punctuation."""
import re
import unicodedata


def plain_speech(text):
    text = text.replace('’', "'")
    text = re.sub(r"\bI'm\b", 'I am', text, flags=re.I)
    text = text.replace("'", '')
    text = unicodedata.normalize('NFKD', text)
    text = ''.join(c for c in text if not unicodedata.combining(c))
    return ' '.join(re.sub(r'[^a-zA-Z0-9\s]', ' ', text).split())
