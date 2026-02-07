"""
Personalization API - Adjust content based on user experience level
Uses Modular AgentService (OpenAI/Gemini) to rewrite content
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from app.services.agent_service import agent_service

router = APIRouter()

class PersonalizeRequest(BaseModel):
    content: str
    software_experience: str
    hardware_experience: str


class PersonalizeResponse(BaseModel):
    personalized_content: str
    adjustments_made: list[str]


@router.post("/", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest):
    """
    Personalize content based on user's experience level
    """

    try:
        # Build personalization prompt
        system_prompt = "You are an expert technical educator."
        
        user_prompt = f"""Adjust the following educational content to match the reader's experience level:

Software Experience: {request.software_experience}
Hardware Experience: {request.hardware_experience}

Guidelines:
- For Beginners: Add more explanations, use analogies, define technical terms
- For Intermediate: Balance technical depth with clarity
- For Advanced: Be concise, use advanced terminology

Preserve all code examples and links.
Return adjusted content in Markdown.

Content to personalize:
{request.content}"""

        # Call Agent Service to personalize
        personalized_content = agent_service.generate_llm_response(
            system_prompt=system_prompt,
            user_prompt=user_prompt,
            temperature=0.7
        )

        # Determine adjustments made (Static logic for UI feedback)
        adjustments = []
        if request.software_experience == "Beginner":
            adjustments.append("Added detailed explanations for software concepts")
        elif request.software_experience == "Advanced":
            adjustments.append("Increased technical depth for software")

        if request.hardware_experience == "Beginner":
            adjustments.append("Simplified hardware terminology")
        elif request.hardware_experience == "Advanced":
            adjustments.append("Focused on advanced hardware details")

        return PersonalizeResponse(
            personalized_content=personalized_content,
            adjustments_made=adjustments
        )

    except Exception as e:
        print(f"Personalization error: {e}")
        raise HTTPException(status_code=500, detail=f"Personalization failed: {str(e)}")
