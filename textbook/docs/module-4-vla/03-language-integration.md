---
sidebar_position: 4
title: "Language Integration"
description: "Processing natural language instructions"
keywords: [nlp, llm, instruction following, grounding, natural language, transformers, language models, embeddings, action planning, multimodal, phi-2]
---

# Language Integration

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Integrate language models with robot control",
    "Process and ground natural language instructions",
    "Handle ambiguity and clarification requests",
    "Optimize LLM inference on Jetson"
  ]}
/>

## Language Models for Robotics

VLA systems use language models to understand instructions and generate action plans.

```python title="language_processor.py" showLineNumbers
from transformers import AutoTokenizer, AutoModelForCausalLM

class LanguageProcessor:
    def __init__(self):
        self.tokenizer = AutoTokenizer.from_pretrained(
            "microsoft/phi-2"
        )
        self.model = AutoModelForCausalLM.from_pretrained(
            "microsoft/phi-2",
            torch_dtype=torch.float16
        )

    def process_instruction(self, text, visual_context):
        prompt = f"""
        Visual Scene: {visual_context}
        Instruction: {text}
        Action:
        """

        inputs = self.tokenizer(prompt, return_tensors="pt")
        outputs = self.model.generate(
            **inputs,
            max_new_tokens=50
        )

        return self.tokenizer.decode(outputs[0])
```

## Grounding Language to Actions

```python title="action_grounding.py" showLineNumbers
def ground_to_action(instruction, objects):
    """
    Map natural language to robot primitives
    """
    action_map = {
        "pick up": "grasp",
        "move to": "navigate",
        "place": "release",
        "find": "search"
    }

    # Parse instruction
    for phrase, action in action_map.items():
        if phrase in instruction.lower():
            target = extract_target_object(instruction, objects)
            return {"action": action, "target": target}

    return None
```

## Summary

- Language models decode human instructions
- Grounding maps language to robot actions
- Multimodal reasoning improves task understanding
