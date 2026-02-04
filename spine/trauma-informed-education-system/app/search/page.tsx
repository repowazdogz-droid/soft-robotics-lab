import { getDocsIndex } from "@/lib/docsIndex";
import SearchContent from "./SearchContent";

export default function SearchPage() {
  const allItems = getDocsIndex();
  return (
    <div className="mx-auto max-w-3xl px-6 py-10 space-y-3">
      <h1 className="text-2xl font-semibold">Search</h1>
      <SearchContent allItems={allItems} />
    </div>
  );
}
