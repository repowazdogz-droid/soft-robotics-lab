interface Props {
  title: string;
  id: string;
  children: React.ReactNode;
  className?: string;
}

export function SectionCard({ title, id, children, className = '' }: Props) {
  return (
    <section className={`p-6 border border-slate-200 rounded-lg bg-white ${className}`}>
      <h3 className="text-lg font-medium text-slate-800 mb-4">
        <span className="text-slate-400 mr-2">{id}.</span>
        {title}
      </h3>
      {children}
    </section>
  );
}
