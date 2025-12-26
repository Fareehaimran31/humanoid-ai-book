---
sidebar_position: 2
title: "Language Processing for Voice Commands"
---

# Language Processing for Voice Commands

## Concept Overview

Language processing in robotics enables robots to understand and respond to natural language commands, significantly improving human-robot interaction. This chapter covers the integration of speech recognition, natural language understanding, and command execution systems. We'll explore both offline and online approaches, covering everything from basic voice command recognition to complex natural language processing for sophisticated robot behaviors.

## Architecture Diagram: Language Processing System

```
┌─────────────────────────────────────────────────────────────────┐
│                 Language Processing Architecture                │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐ │
│  │   Speech        │    │   Natural       │    │   Command   │ │
│  │   Recognition   │───▶│   Language      │───▶│   Mapping   │ │
│  │   (STT)         │    │   Understanding │    │   & Action  │ │
│  │   (Whisper,     │    │   (Intent,      │    │   Execution │ │
│  │   Vosk)         │    │   Entities)     │    │   (ROS 2)   │ │
│  └─────────────────┘    └─────────────────┘    └─────────────┘ │
│              │                   │                    │        │
│              └───────────┐     ┌─┘                  ┌─┘        │
│                          ▼     ▼                  ▼            │
│                    ┌─────────────────────────────────────────┐ │
│                    │      Language Processing              │ │
│                    │   Pipeline (Streaming, Batching)      │ │
│                    └─────────────────────────────────────────┘ │
│                                   │                           │
│                                   ▼                           │
│                    ┌─────────────────────────────────────────┐ │
│                    │      Integration Layer                │ │
│                    │   (ROS 2, Isaac ROS, Dialogue Mgmt)   │ │
│                    └─────────────────────────────────────────┘ │
│                                   │                           │
│                                   ▼                           │
│                    ┌─────────────────────────────────────────┐ │
│                    │      Robot Control Layer              │ │
│                    │   (Navigation, Manipulation, Actions) │ │
│                    └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Speech Recognition Systems

### Offline Speech Recognition with Vosk

Vosk is an excellent choice for offline speech recognition in robotics applications:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import pyaudio
import wave
import json
from vosk import Model, KaldiRecognizer
import queue
import threading

class VoskSpeechRecognizer(Node):
    def __init__(self):
        super().__init__('vosk_speech_recognizer')

        # Initialize Vosk model (requires pre-downloaded model)
        try:
            self.model = Model(lang="en-us")  # Download model files separately
            self.rec = KaldiRecognizer(self.model, 16000)
            self.rec.SetWords(True)
        except Exception as e:
            self.get_logger().error(f'Failed to load Vosk model: {e}')
            self.model = None
            self.rec = None

        # Create publisher for recognized text
        self.text_pub = self.create_publisher(String, '/speech/text', 10)

        # Audio parameters
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

        # Audio queue for processing
        self.audio_queue = queue.Queue()

        # Initialize audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = None

        # Start audio processing thread
        self.audio_thread = None
        self.running = False

        # Command mapping
        self.command_map = {
            'move forward': 'forward',
            'go forward': 'forward',
            'move backward': 'backward',
            'go backward': 'backward',
            'turn left': 'left',
            'turn right': 'right',
            'stop': 'stop',
            'go to kitchen': 'kitchen',
            'go to living room': 'living_room',
            'pick up object': 'pick_up',
            'drop object': 'drop',
            'find person': 'find_person',
            'follow me': 'follow',
            'what time is it': 'time',
            'what is your name': 'name'
        }

        self.get_logger().info('Vosk Speech Recognizer initialized')

    def start_recognition(self):
        """Start the speech recognition process."""
        if not self.model:
            self.get_logger().error('Cannot start recognition: no model loaded')
            return

        # Open audio stream
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        self.running = True

        # Start audio processing thread
        self.audio_thread = threading.Thread(target=self.audio_processing_loop)
        self.audio_thread.start()

        self.get_logger().info('Speech recognition started')

    def stop_recognition(self):
        """Stop the speech recognition process."""
        self.running = False

        if self.audio_thread:
            self.audio_thread.join()

        if self.stream:
            self.stream.stop_stream()
            self.stream.close()

        self.audio.terminate()

        self.get_logger().info('Speech recognition stopped')

    def audio_processing_loop(self):
        """Process audio in a separate thread."""
        while self.running:
            try:
                # Read audio data
                data = self.stream.read(self.chunk, exception_on_overflow=False)

                # Add to queue for processing
                self.audio_queue.put(data)

                # Process available audio data
                if self.rec.AcceptWaveform(data):
                    result = self.rec.Result()
                    self.process_recognition_result(result)

            except Exception as e:
                self.get_logger().error(f'Error in audio processing: {e}')
                break

    def process_recognition_result(self, result_json):
        """Process the recognition result."""
        try:
            result = json.loads(result_json)
            if 'text' in result and result['text']:
                recognized_text = result['text'].lower().strip()

                self.get_logger().info(f'Recognized: {recognized_text}')

                # Publish recognized text
                text_msg = String()
                text_msg.data = recognized_text
                self.text_pub.publish(text_msg)

                # Map to command
                command = self.map_command(recognized_text)
                if command:
                    self.execute_command(command)

        except json.JSONDecodeError:
            self.get_logger().error('Failed to decode recognition result')

    def map_command(self, text):
        """Map recognized text to robot commands."""
        for key_phrase, command in self.command_map.items():
            if key_phrase in text:
                return command

        # Handle more complex patterns
        if 'move' in text or 'go' in text:
            if 'forward' in text or 'ahead' in text:
                return 'forward'
            elif 'backward' in text or 'back' in text:
                return 'backward'
            elif 'left' in text:
                return 'left'
            elif 'right' in text:
                return 'right'

        return None

    def execute_command(self, command):
        """Execute the mapped command."""
        self.get_logger().info(f'Executing command: {command}')

        # In a real implementation, this would publish to appropriate topics
        # For now, just log the command
        if command in ['forward', 'backward', 'left', 'right', 'stop']:
            self.get_logger().info(f'Motion command: {command}')
        elif command in ['kitchen', 'living_room']:
            self.get_logger().info(f'Navigation command to: {command}')
        elif command in ['pick_up', 'drop']:
            self.get_logger().info(f'Manipulation command: {command}')
        else:
            self.get_logger().info(f'Other command: {command}')

def main(args=None):
    rclpy.init(args=args)
    recognizer = VoskSpeechRecognizer()

    try:
        recognizer.start_recognition()
        rclpy.spin(recognizer)
    except KeyboardInterrupt:
        recognizer.get_logger().info('Interrupted by user')
    finally:
        recognizer.stop_recognition()
        recognizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Online Speech Recognition with Whisper

OpenAI's Whisper model provides state-of-the-art speech recognition:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import torch
import whisper
import numpy as np
import io
from scipy.io import wavfile
import tempfile
import os

class WhisperSpeechRecognizer(Node):
    def __init__(self):
        super().__init__('whisper_speech_recognizer')

        # Load Whisper model
        try:
            # Use 'tiny' model for faster processing, 'base' for better accuracy
            self.model = whisper.load_model("base")
            self.get_logger().info('Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            self.model = None

        # Create subscriber for audio data
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_data',
            self.audio_callback,
            10
        )

        # Create publisher for recognized text
        self.text_pub = self.create_publisher(String, '/speech/text', 10)

        # Processing parameters
        self.sample_rate = 16000  # Whisper expects 16kHz audio
        self.processing_queue = []

        self.get_logger().info('Whisper Speech Recognizer initialized')

    def audio_callback(self, msg):
        """Process incoming audio data."""
        if not self.model:
            return

        try:
            # Convert audio data to numpy array
            # Assuming audio data is in 16-bit PCM format
            audio_array = np.frombuffer(msg.data, dtype=np.int16)

            # Normalize to float32
            audio_array = audio_array.astype(np.float32) / 32768.0

            # Resample if necessary (Whisper expects 16kHz)
            if msg.info.sample_rate != self.sample_rate:
                # Simple resampling (in practice, use proper resampling)
                factor = self.sample_rate / msg.info.sample_rate
                new_length = int(len(audio_array) * factor)
                indices = (np.arange(new_length) * (len(audio_array) / new_length)).astype(int)
                audio_array = audio_array[indices]

            # Add to processing queue
            self.processing_queue.append(audio_array)

            # Process if we have enough audio (e.g., 5 seconds of audio)
            if len(audio_array) >= self.sample_rate * 5:
                self.process_audio_batch()

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def process_audio_batch(self):
        """Process accumulated audio data with Whisper."""
        if not self.processing_queue:
            return

        # Concatenate all audio segments
        full_audio = np.concatenate(self.processing_queue)

        # Process with Whisper
        try:
            # Convert to tensor
            audio_tensor = torch.from_numpy(full_audio).float()

            # Transcribe
            result = self.model.transcribe(audio_tensor.numpy())
            recognized_text = result['text'].strip()

            if recognized_text:
                self.get_logger().info(f'Whisper recognized: {recognized_text}')

                # Publish recognized text
                text_msg = String()
                text_msg.data = recognized_text
                self.text_pub.publish(text_msg)

                # Process the command
                self.process_command(recognized_text)

            # Clear the queue
            self.processing_queue = []

        except Exception as e:
            self.get_logger().error(f'Error in Whisper transcription: {e}')

    def process_command(self, text):
        """Process the recognized command."""
        # Extract intents and entities from the text
        intent, entities = self.parse_intent(text)

        self.get_logger().info(f'Parsed intent: {intent}, entities: {entities}')

        # Execute based on intent
        if intent == 'navigation':
            self.execute_navigation(entities)
        elif intent == 'manipulation':
            self.execute_manipulation(entities)
        elif intent == 'information':
            self.execute_information(entities)
        else:
            self.get_logger().info(f'Unknown intent for: {text}')

    def parse_intent(self, text):
        """Parse intent and entities from text."""
        text_lower = text.lower()

        # Define intent patterns
        navigation_keywords = ['go to', 'move to', 'navigate to', 'go', 'move', 'navigate']
        manipulation_keywords = ['pick up', 'grab', 'take', 'drop', 'put down', 'place']
        information_keywords = ['what', 'how', 'when', 'where', 'who', 'time', 'name', 'help']

        # Determine intent
        if any(keyword in text_lower for keyword in navigation_keywords):
            intent = 'navigation'
            # Extract location entity
            entities = self.extract_location(text_lower)
        elif any(keyword in text_lower for keyword in manipulation_keywords):
            intent = 'manipulation'
            # Extract object entity
            entities = self.extract_object(text_lower)
        elif any(keyword in text_lower for keyword in information_keywords):
            intent = 'information'
            entities = {'query': text}
        else:
            intent = 'unknown'
            entities = {}

        return intent, entities

    def extract_location(self, text):
        """Extract location entities from text."""
        locations = ['kitchen', 'living room', 'bedroom', 'bathroom', 'office', 'dining room', 'hallway']
        found_locations = [loc for loc in locations if loc in text]

        return {'locations': found_locations}

    def extract_object(self, text):
        """Extract object entities from text."""
        objects = ['object', 'item', 'cup', 'bottle', 'book', 'phone', 'keys', 'toy']
        found_objects = [obj for obj in objects if obj in text]

        return {'objects': found_objects}

    def execute_navigation(self, entities):
        """Execute navigation command."""
        if 'locations' in entities and entities['locations']:
            location = entities['locations'][0]
            self.get_logger().info(f'Navigating to {location}')
            # In real implementation, publish navigation goal
        else:
            self.get_logger().info('Navigation command without location')

    def execute_manipulation(self, entities):
        """Execute manipulation command."""
        if 'objects' in entities and entities['objects']:
            obj = entities['objects'][0]
            self.get_logger().info(f'Manipulation command for {obj}')
            # In real implementation, publish manipulation command
        else:
            self.get_logger().info('Manipulation command without object')

    def execute_information(self, entities):
        """Execute information query."""
        query = entities.get('query', '')
        self.get_logger().info(f'Information query: {query}')
        # In real implementation, respond to query

def main(args=None):
    rclpy.init(args=args)
    recognizer = WhisperSpeechRecognizer()

    try:
        rclpy.spin(recognizer)
    except KeyboardInterrupt:
        pass
    finally:
        recognizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Natural Language Understanding

### Intent Classification with Transformers

```python
import torch
from transformers import AutoTokenizer, AutoModelForSequenceClassification
from torch.nn.functional import softmax
import numpy as np

class IntentClassifier:
    def __init__(self, model_name="microsoft/DialoGPT-medium"):
        # For intent classification, we might use a different model
        # Here's an example using a pre-trained model for demonstration
        self.tokenizer = AutoTokenizer.from_pretrained("bert-base-uncased")

        # Define intent classes
        self.intents = [
            'greeting',
            'navigation',
            'manipulation',
            'information_request',
            'stop',
            'follow',
            'find_object',
            'find_person'
        ]

        # In practice, you would train your own classifier
        # For now, using a simple keyword-based approach
        self.intent_keywords = {
            'greeting': ['hello', 'hi', 'hey', 'greetings'],
            'navigation': ['go to', 'move to', 'navigate', 'go', 'move', 'travel'],
            'manipulation': ['pick up', 'grab', 'take', 'drop', 'put', 'place'],
            'information_request': ['what', 'how', 'when', 'where', 'who', 'time', 'name'],
            'stop': ['stop', 'halt', 'pause'],
            'follow': ['follow', 'come', 'behind', 'after'],
            'find_object': ['find', 'look for', 'search for', 'locate'],
            'find_person': ['find', 'look for', 'search for', 'locate', 'person', 'people']
        }

    def classify_intent(self, text):
        """Classify intent using keyword matching (simplified approach)."""
        text_lower = text.lower()

        # Score each intent based on keyword matches
        scores = {}
        for intent, keywords in self.intent_keywords.items():
            score = sum(1 for keyword in keywords if keyword in text_lower)
            scores[intent] = score

        # Return the intent with highest score
        if max(scores.values()) > 0:
            return max(scores, key=scores.get)
        else:
            return 'unknown'

    def extract_entities(self, text, intent):
        """Extract entities based on intent."""
        entities = {}

        if intent == 'navigation':
            # Extract location entities
            locations = ['kitchen', 'living room', 'bedroom', 'bathroom', 'office', 'dining room', 'hallway']
            found_locations = [loc for loc in locations if loc in text.lower()]
            entities['locations'] = found_locations

        elif intent == 'manipulation':
            # Extract object entities
            objects = ['cup', 'bottle', 'book', 'phone', 'keys', 'toy', 'object', 'item']
            found_objects = [obj for obj in objects if obj in text.lower()]
            entities['objects'] = found_objects

        elif intent == 'find_object':
            # Extract object to find
            objects = ['cup', 'bottle', 'book', 'phone', 'keys', 'toy']
            found_objects = [obj for obj in objects if obj in text.lower()]
            entities['target_objects'] = found_objects

        return entities
```

## Dialogue Management System

### State-Based Dialogue Manager

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
from collections import deque

class DialogueManager(Node):
    def __init__(self):
        super().__init__('dialogue_manager')

        # Create subscribers and publishers
        self.speech_sub = self.create_subscription(
            String,
            '/speech/text',
            self.speech_callback,
            10
        )

        self.response_pub = self.create_publisher(
            String,
            '/speech/response',
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize dialogue state
        self.dialogue_state = {
            'current_state': 'idle',
            'context': {},
            'history': deque(maxlen=10),  # Keep last 10 interactions
            'user_preferences': {},
            'robot_name': 'Roberto'
        }

        # Define dialogue states
        self.states = {
            'idle': self.handle_idle_state,
            'navigation': self.handle_navigation_state,
            'manipulation': self.handle_manipulation_state,
            'information': self.handle_information_state,
            'follow': self.handle_follow_state
        }

        # Initialize NLU components
        self.intent_classifier = IntentClassifier()

        self.get_logger().info('Dialogue Manager initialized')

    def speech_callback(self, msg):
        """Process incoming speech text."""
        user_input = msg.data.lower().strip()

        self.get_logger().info(f'User said: {user_input}')

        # Add to history
        self.dialogue_state['history'].append({
            'type': 'user',
            'text': user_input,
            'timestamp': self.get_clock().now().to_msg()
        })

        # Classify intent
        intent = self.intent_classifier.classify_intent(user_input)
        entities = self.intent_classifier.extract_entities(user_input, intent)

        self.get_logger().info(f'Classified intent: {intent}, entities: {entities}')

        # Update context
        self.dialogue_state['context']['last_intent'] = intent
        self.dialogue_state['context']['last_entities'] = entities

        # Process based on current state and intent
        response = self.process_dialogue(intent, entities, user_input)

        # Publish response
        if response:
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)

            # Add to history
            self.dialogue_state['history'].append({
                'type': 'system',
                'text': response,
                'timestamp': self.get_clock().now().to_msg()
            })

    def process_dialogue(self, intent, entities, user_input):
        """Process dialogue based on intent and current state."""
        # Handle state transitions
        if intent == 'navigation':
            self.dialogue_state['current_state'] = 'navigation'
        elif intent == 'manipulation':
            self.dialogue_state['current_state'] = 'manipulation'
        elif intent == 'follow':
            self.dialogue_state['current_state'] = 'follow'
        elif intent == 'information_request':
            self.dialogue_state['current_state'] = 'information'
        elif intent == 'stop':
            self.dialogue_state['current_state'] = 'idle'
            self.stop_robot()

        # Process in current state
        if self.dialogue_state['current_state'] in self.states:
            return self.states[self.dialogue_state['current_state']](intent, entities, user_input)
        else:
            return self.handle_idle_state(intent, entities, user_input)

    def handle_idle_state(self, intent, entities, user_input):
        """Handle idle state."""
        if intent == 'greeting':
            return f'Hello! I am {self.dialogue_state["robot_name"]}. How can I help you today?'
        elif intent == 'navigation':
            if 'locations' in entities and entities['locations']:
                location = entities['locations'][0]
                self.start_navigation(location)
                return f'Okay, I am navigating to the {location}.'
            else:
                return 'Where would you like me to go?'
        elif intent == 'manipulation':
            if 'objects' in entities and entities['objects']:
                obj = entities['objects'][0]
                self.start_manipulation(obj)
                return f'Okay, I will try to pick up the {obj}.'
            else:
                return 'What object would you like me to manipulate?'
        elif intent == 'information_request':
            return self.handle_information_query(user_input)
        elif intent == 'follow':
            self.start_follow_mode()
            return 'Okay, I will follow you now.'
        else:
            return f"I'm sorry, I didn't understand that. You can ask me to navigate somewhere, manipulate an object, or ask me questions."

    def handle_navigation_state(self, intent, entities, user_input):
        """Handle navigation state."""
        if intent == 'stop':
            self.stop_robot()
            self.dialogue_state['current_state'] = 'idle'
            return 'I have stopped navigation. How else can I help you?'
        elif intent == 'navigation':
            if 'locations' in entities and entities['locations']:
                location = entities['locations'][0]
                self.start_navigation(location)
                return f'Okay, changing destination to {location}.'
            else:
                return 'Where would you like to go now?'
        else:
            return 'I am currently navigating. Say "stop" if you want me to stop.'

    def handle_manipulation_state(self, intent, entities, user_input):
        """Handle manipulation state."""
        if intent == 'stop':
            self.stop_manipulation()
            self.dialogue_state['current_state'] = 'idle'
            return 'I have stopped manipulation. How else can I help you?'
        else:
            return 'I am currently trying to manipulate an object. Say "stop" if you want me to stop.'

    def handle_information_state(self, intent, entities, user_input):
        """Handle information state."""
        response = self.handle_information_query(user_input)
        self.dialogue_state['current_state'] = 'idle'  # Return to idle after response
        return response

    def handle_follow_state(self, intent, entities, user_input):
        """Handle follow state."""
        if intent == 'stop':
            self.stop_follow_mode()
            self.dialogue_state['current_state'] = 'idle'
            return 'I have stopped following. How else can I help you?'
        else:
            return 'I am currently following you. Say "stop" if you want me to stop.'

    def handle_information_query(self, user_input):
        """Handle information queries."""
        user_input_lower = user_input.lower()

        if 'time' in user_input_lower:
            from datetime import datetime
            current_time = datetime.now().strftime("%H:%M")
            return f'The current time is {current_time}.'
        elif 'name' in user_input_lower:
            return f'My name is {self.dialogue_state["robot_name"]}.'
        elif 'help' in user_input_lower:
            return 'I can help you with navigation, object manipulation, information, and following. For example, you can say "go to kitchen", "pick up the cup", or "what time is it".'
        elif 'yourself' in user_input_lower or 'you' in user_input_lower:
            return 'I am a service robot designed to assist with various tasks. I can navigate, manipulate objects, and answer questions.'
        else:
            return 'I can tell you the time, my name, or provide help information. What would you like to know?'

    def start_navigation(self, location):
        """Start navigation to a location."""
        # In a real implementation, this would send navigation goals
        self.get_logger().info(f'Starting navigation to {location}')

        # For simulation, just move forward briefly
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.2
        cmd_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def start_manipulation(self, obj):
        """Start manipulation of an object."""
        self.get_logger().info(f'Starting manipulation of {obj}')

    def start_follow_mode(self):
        """Start follow mode."""
        self.get_logger().info('Starting follow mode')

    def stop_robot(self):
        """Stop all robot motion."""
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        self.get_logger().info('Robot stopped')

    def stop_manipulation(self):
        """Stop manipulation."""
        self.get_logger().info('Manipulation stopped')

    def stop_follow_mode(self):
        """Stop follow mode."""
        self.stop_robot()
        self.get_logger().info('Follow mode stopped')

def main(args=None):
    rclpy.init(args=args)
    dialogue_manager = DialogueManager()

    try:
        rclpy.spin(dialogue_manager)
    except KeyboardInterrupt:
        pass
    finally:
        dialogue_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with ROS 2 and Isaac

### Isaac ROS Language Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class IsaacLanguageIntegration(Node):
    def __init__(self):
        super().__init__('isaac_language_integration')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscribers
        self.speech_sub = self.create_subscription(
            String,
            '/speech/text',
            self.speech_callback,
            10
        )

        self.vision_sub = self.create_subscription(
            Image,
            '/front_stereo_camera/left/image_rect_color',
            self.vision_callback,
            10
        )

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.response_pub = self.create_publisher(String, '/speech/response', 10)

        # Robot state
        self.last_vision_data = None
        self.is_executing_command = False

        # Command queue
        self.command_queue = []

        self.get_logger().info('Isaac Language Integration initialized')

    def speech_callback(self, msg):
        """Process speech commands."""
        command = msg.data.lower().strip()

        self.get_logger().info(f'Received speech command: {command}')

        # Parse and queue command
        parsed_command = self.parse_speech_command(command)
        if parsed_command:
            self.command_queue.append(parsed_command)
            self.execute_next_command()

    def vision_callback(self, msg):
        """Process vision data."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_vision_data = cv_image

            # Process image for object detection if needed
            # This could be used to validate commands or provide visual feedback
        except Exception as e:
            self.get_logger().error(f'Error processing vision: {e}')

    def parse_speech_command(self, command):
        """Parse speech command into executable actions."""
        # Define command patterns
        if 'move forward' in command or 'go forward' in command:
            return {'type': 'motion', 'action': 'forward', 'duration': 2.0}
        elif 'move backward' in command or 'go backward' in command:
            return {'type': 'motion', 'action': 'backward', 'duration': 2.0}
        elif 'turn left' in command:
            return {'type': 'motion', 'action': 'left', 'duration': 1.0}
        elif 'turn right' in command:
            return {'type': 'motion', 'action': 'right', 'duration': 1.0}
        elif 'stop' in command:
            return {'type': 'motion', 'action': 'stop'}
        elif 'find' in command and ('person' in command or 'people' in command):
            return {'type': 'vision', 'action': 'find_person'}
        elif 'find' in command and ('object' in command or 'cup' in command or 'bottle' in command):
            return {'type': 'vision', 'action': 'find_object', 'target': self.extract_object(command)}
        else:
            return None

    def extract_object(self, command):
        """Extract object from command."""
        objects = ['cup', 'bottle', 'book', 'phone', 'keys', 'toy']
        for obj in objects:
            if obj in command:
                return obj
        return 'object'  # default

    def execute_next_command(self):
        """Execute the next command in the queue."""
        if not self.command_queue or self.is_executing_command:
            return

        command = self.command_queue.pop(0)
        self.is_executing_command = True

        self.get_logger().info(f'Executing command: {command}')

        if command['type'] == 'motion':
            self.execute_motion_command(command)
        elif command['type'] == 'vision':
            self.execute_vision_command(command)

    def execute_motion_command(self, command):
        """Execute motion command."""
        cmd_msg = Twist()

        if command['action'] == 'forward':
            cmd_msg.linear.x = 0.3
        elif command['action'] == 'backward':
            cmd_msg.linear.x = -0.3
        elif command['action'] == 'left':
            cmd_msg.angular.z = 0.5
        elif command['action'] == 'right':
            cmd_msg.angular.z = -0.5
        elif command['action'] == 'stop':
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_msg)

        # If it's a timed command, stop after duration
        if command['action'] != 'stop' and 'duration' in command:
            self.create_timer(command['duration'], self.stop_motion)

    def execute_vision_command(self, command):
        """Execute vision command."""
        if command['action'] == 'find_person':
            self.find_person()
        elif command['action'] == 'find_object':
            self.find_object(command['target'])

    def find_person(self):
        """Find person using vision system."""
        if self.last_vision_data is not None:
            # Process the image to find people
            # This is a simplified example
            image = self.last_vision_data

            # In a real implementation, use person detection
            # For now, just respond that we're looking
            response_msg = String()
            response_msg.data = 'I am looking for a person. Please wait.'
            self.response_pub.publish(response_msg)

            # In practice, you would use a person detection model
            # and provide feedback when a person is found

    def find_object(self, target_object):
        """Find specific object using vision system."""
        if self.last_vision_data is not None:
            response_msg = String()
            response_msg.data = f'I am looking for the {target_object}. Please wait.'
            self.response_pub.publish(response_msg)

    def stop_motion(self):
        """Stop robot motion."""
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        self.is_executing_command = False

        # Execute next command if available
        self.execute_next_command()

def main(args=None):
    rclpy.init(args=args)
    isaac_lang_node = IsaacLanguageIntegration()

    try:
        rclpy.spin(isaac_lang_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on shutdown
        cmd_msg = Twist()
        isaac_lang_node.cmd_vel_pub.publish(cmd_msg)
        isaac_lang_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Text-to-Speech Integration

### Simple TTS Response System

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import threading
import queue

class TTSResponseNode(Node):
    def __init__(self):
        super().__init__('tts_response_node')

        # Initialize TTS engine
        try:
            self.tts_engine = pyttsx3.init()

            # Configure TTS properties
            self.tts_engine.setProperty('rate', 150)  # Speed of speech
            self.tts_engine.setProperty('volume', 0.9)  # Volume level (0.0 to 1.0)

            # Get available voices
            voices = self.tts_engine.getProperty('voices')
            if voices:
                # Use the first available voice (usually female in many systems)
                self.tts_engine.setProperty('voice', voices[0].id)

        except Exception as e:
            self.get_logger().error(f'Failed to initialize TTS engine: {e}')
            self.tts_engine = None

        # Create subscriber for response text
        self.response_sub = self.create_subscription(
            String,
            '/speech/response',
            self.response_callback,
            10
        )

        # Text queue for processing
        self.text_queue = queue.Queue()
        self.speaking_thread = None
        self.running = False

        # Start speaking thread
        self.start_speaking_thread()

        self.get_logger().info('TTS Response Node initialized')

    def start_speaking_thread(self):
        """Start the speaking thread."""
        self.running = True
        self.speaking_thread = threading.Thread(target=self.speaking_loop)
        self.speaking_thread.start()

    def response_callback(self, msg):
        """Process response text."""
        if self.tts_engine:
            # Add to queue for speaking
            self.text_queue.put(msg.data)

    def speaking_loop(self):
        """Process text in speaking queue."""
        while self.running:
            try:
                # Get text from queue (with timeout to allow checking self.running)
                text = self.text_queue.get(timeout=1.0)

                if text:
                    self.get_logger().info(f'Speaking: {text}')

                    # Speak the text
                    if self.tts_engine:
                        self.tts_engine.say(text)
                        self.tts_engine.runAndWait()

            except queue.Empty:
                # Timeout occurred, continue loop to check self.running
                continue
            except Exception as e:
                self.get_logger().error(f'Error in speaking loop: {e}')
                continue

    def destroy_node(self):
        """Clean up before destroying node."""
        self.running = False
        if self.speaking_thread:
            self.speaking_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSResponseNode()

    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        pass
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab Instructions

### Exercise 1: Speech Recognition Setup

1. Install Vosk speech recognition library
2. Download an English language model for Vosk
3. Implement the speech recognition node
4. Test with various voice commands
5. Evaluate recognition accuracy in different noise conditions

### Exercise 2: Natural Language Processing

1. Create a dialogue manager that can handle multiple intents
2. Implement entity extraction for locations and objects
3. Test the system with complex voice commands
4. Add context awareness to handle follow-up questions

### Exercise 3: Voice Command Integration

1. Integrate speech recognition with robot navigation
2. Create voice commands for robot control
3. Implement a simple dialogue system
4. Test the complete voice-controlled robot system

## Troubleshooting Language Systems

### Common Issues and Solutions

1. **Poor Recognition Accuracy**:
   - Improve microphone quality and positioning
   - Adjust speech recognition parameters
   - Use noise reduction techniques
   - Train custom acoustic models if needed

2. **Latency Issues**:
   - Optimize processing pipeline
   - Use streaming recognition instead of batch processing
   - Consider edge computing for real-time processing

3. **Context Confusion**:
   - Implement proper dialogue state management
   - Use context-aware NLU models
   - Add confirmation steps for critical commands

## Learning Objectives

After completing this chapter, you should be able to:
- Implement offline and online speech recognition systems for robotics
- Design natural language understanding pipelines for intent classification
- Create dialogue management systems for human-robot interaction
- Integrate voice commands with robot control systems
- Apply Isaac ROS language processing packages
- Implement text-to-speech systems for robot responses
- Troubleshoot common issues in voice processing for robotics