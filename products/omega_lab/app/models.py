"""SQLAlchemy models for OMEGA Lab Cognition Server."""

from datetime import datetime

from sqlalchemy import Column, DateTime, Float, ForeignKey, Integer, String, Text
from sqlalchemy.orm import declarative_base

Base = declarative_base()


class Hypothesis(Base):
    """A testable claim with confidence tracked by evidence."""

    __tablename__ = "hypotheses"

    id = Column(String(64), primary_key=True)
    claim = Column(Text())
    status = Column(String(32), default="active")
    confidence = Column(Float())
    schema_version = Column(String(16), default="1.0")
    created_at = Column(DateTime(), default=datetime.utcnow)
    updated_at = Column(DateTime(), default=datetime.utcnow, onupdate=datetime.utcnow)


class Evidence(Base):
    """One piece of evidence linking a run to a hypothesis."""

    __tablename__ = "evidence"

    id = Column(Integer, primary_key=True, autoincrement=True)
    hypothesis_id = Column(String(64), ForeignKey("hypotheses.id"))
    run_id = Column(String(128))
    direction = Column(String(32))
    strength = Column(Float())
    rationale = Column(Text())
    schema_version = Column(String(16), default="1.0")
    timestamp = Column(DateTime())


class Run(Base):
    """A single simulation/experiment run."""

    __tablename__ = "runs"

    run_id = Column(String(128), primary_key=True)
    parent_run_id = Column(String(128), nullable=True)
    batch_id = Column(String(128), nullable=True)
    schema_version = Column(String(16), default="1.0")
    design_id = Column(String(128), nullable=True)
    hypothesis_id = Column(String(64))
    experiment_id = Column(String(128))
    engine = Column(String(64))
    timestamp = Column(DateTime())
    environment = Column(String(64))
    metrics_json = Column(Text())
    artifacts_path = Column(String(512))
    notes = Column(Text())
