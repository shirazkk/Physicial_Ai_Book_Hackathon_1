from fastapi import APIRouter, HTTPException
from typing import Optional, Dict, Any, List
from pydantic import BaseModel

from src.utils.config import get_settings
from src.retrieval.search_engine import get_retrieval_engine
from src.context.context_assembler import get_context_assembler
from src.generation.gemini_client import get_gemini_chat_client
from src.generation.system_prompts import get_default_system_prompt, get_selected_text_system_prompt
from src.generation.refusal_handler import get_refusal_handler
from src.models.chat_session import ChatSession
from src.models.chat_message import ChatMessage
from src.indexing.pipeline import get_indexing_pipeline

# Remove unused import
# from fastapi import Depends

# Pydantic models for request/response
class ChatRequest(BaseModel):
    question: str
    session_id: Optional[str] = None
    selected_text_only: bool = False
    selected_text: Optional[str] = None


class ChatResponse(BaseModel):
    response: str
    session_id: str
    citations: Optional[List[Dict[str, Any]]] = None


class IndexRequest(BaseModel):
    directory_path: str = "my-website/docs"


class IndexResponse(BaseModel):
    success: bool
    message: str
    processed_files: int


# Create the router
router = APIRouter(prefix="/api/v1", tags=["chat"])

# Global instances (would be dependency injected in production)
retrieval_engine = get_retrieval_engine()
context_assembler = get_context_assembler()
chat_client = get_gemini_chat_client()
refusal_handler = get_refusal_handler()
indexing_pipeline = get_indexing_pipeline()

# In-memory session storage (would be replaced with database in production)
sessions: Dict[str, ChatSession] = {}


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest) -> ChatResponse:
    """
    Main chat endpoint for the RAG chatbot.
    """
    try:
        # Get or create session
        session_id = request.session_id or f"session_{len(sessions) + 1}"

        if session_id not in sessions:
            from uuid import uuid4
            session_id = str(uuid4())
            sessions[session_id] = ChatSession(session_id=session_id)

        session = sessions[session_id]

        # Determine system prompt based on mode
        if request.selected_text_only and request.selected_text:
            system_prompt = get_selected_text_system_prompt()
        else:
            system_prompt = get_default_system_prompt("Textbook")

        # Retrieve relevant context based on mode
        if request.selected_text_only and request.selected_text:
            # Use only the selected text as context
            retrieved_chunks = retrieval_engine.retrieve_with_selected_text_only(
                selected_text=request.selected_text
            )
        else:
            # Retrieve from knowledge base
            retrieved_chunks = retrieval_engine.retrieve_and_rank(
                query=request.question,
                top_k=5,
                min_score=0.3,
                max_tokens=2048
            )

        # Assemble context
        if request.selected_text_only and request.selected_text:
            context = context_assembler.assemble_context_for_selected_text_only(
                query=request.question,
                selected_text=request.selected_text,
                conversation_history=session.get_messages(),
                system_prompt=system_prompt
            )
        else:
            context = context_assembler.assemble_context(
                query=request.question,
                retrieved_chunks=retrieved_chunks,
                conversation_history=session.get_messages(),
                system_prompt=system_prompt
            )

        # Generate response
        if retrieved_chunks:  # If we have context to work with
            response_result = chat_client.generate_response_with_citations(
                prompt=request.question,
                context=context
            )
            response_text = response_result['response']
            citations = response_result['citations']
        else:
            # No relevant context found, use refusal handler
            response_text = refusal_handler.handle_out_of_corpus_query(
                query=request.question,
                selected_text_mode=request.selected_text_only
            )
            citations = []

        # Add user message to session
        user_msg = ChatMessage(
            message_id=f"user_{len(session.messages)+1}",
            role="user",
            content=request.question
        )
        session.add_message(user_msg)

        # Add assistant response to session
        assistant_msg = ChatMessage(
            message_id=f"assistant_{len(session.messages)+1}",
            role="assistant",
            content=response_text,
            source_context=[chunk for chunk in retrieved_chunks] if retrieved_chunks else []
        )
        session.add_message(assistant_msg)

        return ChatResponse(
            response=response_text,
            session_id=session_id,
            citations=citations
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@router.post("/index")
async def index_documents(request: IndexRequest) -> IndexResponse:
    """
    Endpoint to trigger document indexing.
    """
    try:
        results = indexing_pipeline.index_from_docs_directory(request.directory_path)

        successful_files = sum(1 for success in results.values() if success)

        return IndexResponse(
            success=True,
            message=f"Indexed {successful_files} out of {len(results)} files",
            processed_files=successful_files
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error indexing documents: {str(e)}")


@router.get("/health")
async def health_check():
    """
    Health check endpoint.
    """
    return {"status": "healthy", "service": "rag-chatbot-api"}


@router.get("/sessions/{session_id}")
async def get_session(session_id: str):
    """
    Get a specific session.
    """
    if session_id not in sessions:
        raise HTTPException(status_code=404, detail="Session not found")

    session = sessions[session_id]
    return {
        "session_id": session.session_id,
        "user_id": session.user_id,
        "created_at": session.created_at,
        "updated_at": session.updated_at,
        "message_count": len(session.messages)
    }


@router.delete("/sessions/{session_id}")
async def delete_session(session_id: str):
    """
    Delete a specific session.
    """
    if session_id not in sessions:
        raise HTTPException(status_code=404, detail="Session not found")

    del sessions[session_id]
    return {"message": f"Session {session_id} deleted successfully"}